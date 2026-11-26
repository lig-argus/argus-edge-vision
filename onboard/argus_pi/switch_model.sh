#!/usr/bin/env bash
# 탐지 모델(HEF)을 바꾼다 (2026-10-30). config/runtime.env의 ARGUS_HEF, ARGUS_LABELS 줄과 assets/model.json을 함께 바꾸고 검사한다.
# 2026-11-03: 점수 기준 세 줄(ARGUS_SCORE_THRESHOLD, ARGUS_CLASS_SCORE_THRESHOLDS, ARGUS_CLASS_SUPPRESS)도 같이 바꾼다.
#   1클래스(tank) 모델: 전체 0.7, 클래스별 기준과 겹침 지우기는 줄 앞에 # 를 붙여 끔
#   3클래스 모델: 전체 0.25, 클래스별 0.5,0.45,0.35, tank:car:0.3
#   프리셋 json에 "env": {"키": "값" 또는 null} 이 있으면 그 값을 쓴다 (null = 끔).
# 실행: ./switch_model.sh            → 지금 모델과 고를 수 있는 모델 목록
#       ./switch_model.sh <이름>     → 그 모델로 바꿈 (다음에 스택을 켤 때부터 적용)
#       짧은 이름 (2026-11-12부터 번호식, 만든 순서. 뒤 _0.5 / _0.3 은 탱크 기준):
#         indoor1   실내 데모 B2 (11-10, 사람 끔)
#         indoor2   실내 데모 A (11-10, 사람 끔)
#         indoor3   실내 A + 장난감 차 (11-12, 사람 끔)
#         outdoor1, outdoor1_0.5, outdoor1_0.3   ft_b1 (10-30)
#         outdoor2_0.5, outdoor2_0.3             outdoor_ep28 (11-10, ft_b1 + 야외 탱크)
#         outdoor3 (= outdoor3_0.5)              outdoor_p10 (11-12, outdoor_ep28 + 야외 사람)
#         v7.3 (= tank_1ch_max, 탱크만), road1, toy1
#       예전 이름(indoor_A, indoor_B2, indoor_TA, ft_b1_0.5, outdoor_ep28_0.5, outdoor_p10 등)도 그대로 됨
# 모델 목록은 assets/model_presets/<이름>.json (path, sha256, labels, classes). HEF 파일은 지우지 않는다.
set -eu
cd "$(dirname "$0")"
P=assets/model_presets
KEYS="ARGUS_HEF|ARGUS_LABELS|ARGUS_SCORE_THRESHOLD|ARGUS_CLASS_SCORE_THRESHOLDS|ARGUS_CLASS_SUPPRESS"
if [[ $# -eq 0 ]]; then
  echo "지금:"; grep -E "^#? ?($KEYS)=" config/runtime.env | sed 's/^/  /'
  echo "고를 수 있는 모델:"
  for f in $P/*.json; do echo "  $(basename "$f" .json)  ($(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print(d["path"], d["classes"])' "$f"))"; done
  echo "짧은 이름: indoor1, indoor2, indoor3, outdoor1, outdoor1_0.5, outdoor1_0.3, outdoor2_0.5, outdoor2_0.3, outdoor3, v7.3, road1, toy1"
  exit 0
fi
case "$1" in
  indoor1|indoor_B2|indoorB2|demo_B2) NAME=3cls_demo_B2_1ch_max ;;
  indoor2|indoor_A|indoorA|demo_A) NAME=3cls_demo_A_1ch_max ;;
  indoor3|indoor_TA|indoorTA) NAME=3cls_indoor_TA_1ch_max ;;
  outdoor1|ft_b1) NAME=3cls_ft_b1_1ch_max ;;
  outdoor1_0.5|outdoor1_05|ft_b1_0.5|ft_b1_05) NAME=3cls_ft_b1_1ch_max_t05 ;;
  outdoor1_0.3|outdoor1_03|ft_b1_0.3|ft_b1_03) NAME=3cls_ft_b1_1ch_max_t03 ;;
  outdoor2_0.5|outdoor2_05|outdoor_ep28_0.5|outdoor_ep28_05|ep28_0.5) NAME=3cls_outdoor_ep28_1ch_max_t05 ;;
  outdoor2_0.3|outdoor2_03|outdoor_ep28_0.3|outdoor_ep28_03|ep28_0.3) NAME=3cls_outdoor_ep28_1ch_max_t03 ;;
  outdoor3|outdoor3_0.5|outdoor3_05|outdoor_p10|outdoorp10|p10) NAME=3cls_outdoor_p10_1ch_max ;;
  v7.3|v73|tank) NAME=tank_1ch_max ;;
  road1) NAME=3cls_road1_1ch_max ;;
  toy1) NAME=3cls_toy1_1ch_max ;;
  *) NAME=$1 ;;
esac
F=$P/$NAME.json
[[ -f $F ]] || { echo "없는 이름: $1"; exit 1; }
read -r HEF LABELS < <(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print(d["path"], d["labels"])' "$F")
[[ -f $HEF && -f $LABELS ]] || { echo "파일 없음: $HEF 또는 $LABELS"; exit 1; }
cp config/runtime.env "config/runtime.env.before_switch"
cp assets/model.json "assets/model.json.before_switch"
python3 - "$F" <<'PY'
import json, re, sys
d = json.load(open(sys.argv[1]))
if len(d["classes"]) == 1:
    env = {"ARGUS_SCORE_THRESHOLD": "0.7", "ARGUS_CLASS_SCORE_THRESHOLDS": None, "ARGUS_CLASS_SUPPRESS": None}
else:
    env = {"ARGUS_SCORE_THRESHOLD": "0.25", "ARGUS_CLASS_SCORE_THRESHOLDS": "0.5,0.45,0.35",
           "ARGUS_CLASS_SUPPRESS": "tank:car:0.3"}
env.update(d.get("env", {}))
env.update(ARGUS_HEF=d["path"], ARGUS_LABELS=d["labels"])
lines = open("config/runtime.env").read().split("\n")
for key, value in env.items():
    pat = re.compile(r"^(# ?)?" + key + r"=(.*)$")
    hits = [i for i, line in enumerate(lines) if pat.match(line)]
    if not hits:
        sys.exit(f"runtime.env에 {key} 줄이 없음")
    active = [i for i in hits if not lines[i].startswith("#")]
    i = active[0] if active else hits[0]
    old = pat.match(lines[i]).group(2).split("  (")[0].strip()
    if value is None:
        lines[i] = f"# {key}={old}  (이 모델에서는 끔. ./switch_model.sh 가 바꿈)"
    else:
        lines[i] = f"{key}={value}"
open("config/runtime.env", "w").write("\n".join(lines))
d.pop("note", None); d.pop("env", None)
json.dump(d, open("assets/model.json", "w"), ensure_ascii=False, indent=2)
open("assets/model.json", "a").write("\n")
PY
echo "바꿈: $NAME"
grep -E "^#? ?($KEYS)=" config/runtime.env
python3 check_target.py --stage config 2>&1 | grep -E '"checks_passed"|FAIL|Error' || true
echo "지금 켜져 있는 탐지에는 적용되지 않음. 지상국에서 ARGUS 프로세스를 멈추고 영상 연결을 다시 누르면 적용."
