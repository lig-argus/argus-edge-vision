# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""timing_monitor 기록 요약: FC 자세 도착 간격·지연과 CPU 사용률 (무장 구간과 그 밖을 나눠서). 읽기 전용.
사용법: python3 timing_report.py run/timing-monitor/<시작시각> [--all]
"""

import csv, json, sys, datetime
from collections import defaultdict
from pathlib import Path

KST = datetime.timezone(datetime.timedelta(hours=9))


def pct(v, p):
    if not v:
        return float("nan")
    v = sorted(v)
    k = (len(v) - 1) * p / 100
    i = int(k)
    return v[i] if i + 1 >= len(v) else v[i] + (v[i + 1] - v[i]) * (k - i)


def fmt(v, unit="ms"):
    if not v:
        return "-"
    return f"p50 {pct(v, 50):.1f} p90 {pct(v, 90):.1f} p99 {pct(v, 99):.1f} max {max(v):.1f} {unit}"


def main():
    d = Path(sys.argv[1])
    cpu = [json.loads(l) for l in (d / "cpu.jsonl").open() if l.strip()]
    # 무장 구간 (mono_ns 기준)
    armed_spans = []
    start = None
    for r in cpu:
        a = (r.get("state") or {}).get("armed") is True
        if a and start is None:
            start = r["mono_ns"] - 5_000_000_000  # /state는 5초마다 조회
        if not a and start is not None:
            armed_spans.append((start, r["mono_ns"]))
            start = None
    if start is not None:
        armed_spans.append((start, cpu[-1]["mono_ns"]))
    in_armed = lambda t: any(a <= t <= b for a, b in armed_spans)
    w = lambda ns_unix: datetime.datetime.fromtimestamp(ns_unix, KST).strftime("%m-%d %H:%M:%S")
    print(f"기록 {d.name} · CPU 표본 {len(cpu)}초 · 무장 구간 {len(armed_spans)}개")
    for a, b in armed_spans:
        u = next((r["unix_s"] for r in cpu if r["mono_ns"] >= a), None)
        print(f"  무장 {w(u) if u else '?'} 부터 {(b - a) / 1e9:.0f}s")

    groups = [("무장 중", True), ("무장 밖", False)] if armed_spans else [("전체", None)]

    # ---- FC 자세
    rows = []
    with (d / "attitude.csv").open() as f:
        for r in csv.DictReader(f):
            try:
                rows.append(
                    (
                        int(r["monitor_rx_mono_ns"]),
                        int(r["gateway_rx_mono_ns"] or 0),
                        int(float(r["fc_time_boot_ms"]))
                        if r["fc_time_boot_ms"] not in ("", "None")
                        else None,
                        r["session_id"],
                        [
                            float(r[k]) if r[k] not in ("", "None") else 0.0
                            for k in ("roll_rad_s", "pitch_rad_s", "yaw_rad_s")
                        ],
                    )
                )
            except (ValueError, KeyError):
                continue
    print(f"\n[FC 자세 fc/attitude] 표본 {len(rows)}")
    for label, want in groups:
        sel = [r for r in rows if want is None or in_armed(r[0]) == want]
        if len(sel) < 10:
            print(f"  {label}: 표본 {len(sel)}")
            continue
        fc_dt = [
            b[2] - a[2]
            for a, b in zip(sel, sel[1:])
            if a[2] is not None and b[2] is not None and a[3] == b[3] and 0 < b[2] - a[2] < 2000
        ]
        rx_dt = [
            (b[0] - a[0]) / 1e6 for a, b in zip(sel, sel[1:]) if a[3] == b[3] and b[0] - a[0] < 2e9
        ]
        gw2mon = [(r[0] - r[1]) / 1e6 for r in sel if r[1]]
        # FC → 게이트웨이 수신 지연: 오프셋 미상이라 최솟값 기준 상대 지연
        off = [r[1] / 1e6 - r[2] for r in sel if r[1] and r[2] is not None]
        rel = [o - min(off) for o in off] if off else []
        rate = [((x * x + y * y + z * z) ** 0.5) * 57.2958 for _, _, _, _, (x, y, z) in sel]
        dur = (sel[-1][0] - sel[0][0]) / 1e9
        print(f"  {label}: {len(sel)}개 / {dur:.0f}s = 평균 {len(sel) / dur:.1f} Hz")
        print(f"    FC 표본 간격(time_boot_ms) {fmt(fc_dt)}")
        print(
            f"    Pi 도착 간격               {fmt(rx_dt)} · 25ms 초과 {100 * sum(x > 25 for x in rx_dt) / max(1, len(rx_dt)):.1f}% · 50ms 초과 {sum(x > 50 for x in rx_dt)}회 · 100ms 초과 {sum(x > 100 for x in rx_dt)}회"
        )
        print(f"    게이트웨이 수신→발행·도착  {fmt(gw2mon)}  (10 ms 루프 대기 포함)")
        print(f"    FC→게이트웨이 상대 지연     {fmt(rel)}  (최소 지연 대비 추가분)")
        print(
            f"    각속도 크기 {fmt(rate, 'deg/s')} · 40deg/s 초과 {100 * sum(x > 40 for x in rate) / len(rate):.2f}%"
        )

    # ---- CPU
    print("\n[CPU 1초]")
    for label, want in groups:
        sel = [r for r in cpu if want is None or in_armed(r["mono_ns"]) == want]
        if not sel:
            continue
        tot = [r["cores"].get("cpu") for r in sel if r["cores"].get("cpu") is not None]
        cmax = [
            max(v for k, v in r["cores"].items() if k != "cpu" and v is not None)
            for r in sel
            if len(r["cores"]) > 1
        ]
        stall = [r["psi"]["some"]["stall_ms"] for r in sel if "some" in r.get("psi", {})]
        temp = [r["temp_c"] for r in sel if r.get("temp_c") is not None]
        fmin = [min(x for x in r["freq_mhz"] if x) for r in sel if any(r.get("freq_mhz") or [])]
        print(f"  {label} ({len(sel)}s): 전체 {fmt(tot, '%')} · 가장 바쁜 코어 {fmt(cmax, '%')}")
        print(
            f"    CPU 대기(PSI some) {fmt(stall, 'ms/s')} · 온도 최고 {max(temp) if temp else '-'}°C · 최저 클럭 {min(fmin) if fmin else '-'} MHz"
        )
        agg = defaultdict(lambda: dict(cpu=[], delay=[], nv=[]))
        for r in sel:
            for k, v in r["procs"].items():
                name = k.rsplit(":", 1)[0]
                agg[name]["cpu"].append(v["cpu_pct"])
                agg[name]["delay"].append(v["run_delay_ms"])
                agg[name]["nv"].append(v["nvcsw"])
        print(
            f"    {'프로세스':32s} CPU% p50/p95/max      실행대기 ms/s p50/p95/max   비자발전환/s p50"
        )
        for name, v in sorted(agg.items(), key=lambda kv: -pct(kv[1]["cpu"], 50)):
            print(
                f"    {name:32s} {pct(v['cpu'], 50):5.1f}/{pct(v['cpu'], 95):5.1f}/{max(v['cpu']):5.1f}      "
                f"{pct(v['delay'], 50):6.2f}/{pct(v['delay'], 95):6.2f}/{max(v['delay']):7.2f}      {pct(v['nv'], 50):.0f}"
            )


if __name__ == "__main__":
    main()
