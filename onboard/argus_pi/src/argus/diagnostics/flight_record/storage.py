# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""Owned session directories, atomic verified ZIPs and ten-session retention."""

from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import time
import uuid
import zipfile

SCHEMA = "argus.flight.session.v1"


def sync_directory(path):
    """Persist rename/commit order before removing the verified source."""
    try:
        fd = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        # Windows does not expose directory fsync; the Pi runtime uses Linux.
        if os.name != "nt":
            raise


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".next")
    with temp.open("w", encoding="utf8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(temp, path)
    sync_directory(path.parent)


def sha(path):
    with Path(path).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def sanitize(value):
    if isinstance(value, dict):
        result = {}
        for k, v in value.items():
            if any(
                s in str(k).lower()
                for s in ("owner_token", "password", "private_key", "secret", "access_token")
            ):
                result[str(k) + "_present"] = bool(v)
            else:
                result[k] = sanitize(v)
        return result
    if isinstance(value, (list, tuple)):
        return [sanitize(v) for v in value]
    return value


def owned_directory(root, folder, uid):
    root = Path(root).resolve()
    folder = Path(folder)
    if (
        folder.is_symlink()
        or folder.parent.resolve() != root
        or not folder.name.startswith("flight-")
        or not folder.name.endswith(".open")
    ):
        raise ValueError("not an owned session directory")
    marker = folder / "session.json"
    if marker.is_symlink() or not marker.is_file():
        raise ValueError("unsafe session ownership marker")
    metadata = json.loads(marker.read_text(encoding="utf8"))
    if (
        metadata.get("schema") != SCHEMA
        or metadata.get("uid") != uid
        or metadata.get("session_id") != folder.name[:-5]
    ):
        raise ValueError("session ownership mismatch")
    return metadata


def catalog(root, uid):
    result = {}
    root = Path(root)
    for meta in root.glob("flight-*.verified.json"):
        try:
            if meta.is_symlink():
                continue
            m = json.loads(meta.read_text(encoding="utf8"))
            sid = m["session_id"]
            z = root / (sid + ".zip")
            if (
                m.get("schema") != "argus.flight.archive.v1"
                or m.get("uid") != uid
                or m.get("verified") is not True
            ):
                continue
            if (
                not sid.startswith("flight-")
                or Path(sid).name != sid
                or z.is_symlink()
                or not z.is_file()
                or z.stat().st_size != m["zip_bytes"]
            ):
                continue
            result[sid] = dict(
                kind="verified", created_ns=m["created_unix_ns"], zip=z, sidecar=meta
            )
        except (OSError, ValueError, KeyError):
            continue
    for folder in root.glob("flight-*.open"):
        try:
            m = owned_directory(root, folder, uid)
            # Raw data is protected until a verified archive exists; deduplicate the brief commit window.
            # A crash may leave a ZIP and its source together. Source remains protected
            # until the archive worker has verified that the source still matches it.
            result[m["session_id"]] = dict(
                kind="raw", created_ns=m["created_unix_ns"], folder=folder
            )
        except (OSError, ValueError, KeyError):
            if folder.is_dir() and not folder.is_symlink():
                result[folder.name[:-5]] = dict(
                    kind="unverified", created_ns=folder.stat().st_mtime_ns, folder=folder
                )
    return result


def retain(root, uid, limit=10, reserve=0):
    root = Path(root).resolve()
    if not 1 <= limit <= 10 or not 0 <= reserve <= 1:
        raise ValueError("invalid retention budget")
    entries = catalog(root, uid)
    removed = []
    while len(entries) + reserve > limit:
        eligible = sorted(
            ((sid, x) for sid, x in entries.items() if x["kind"] == "verified"),
            key=lambda x: (x[1]["created_ns"], x[0]),
        )
        if not eligible:
            raise RuntimeError(
                "ten unverified/protected sessions occupy storage; new recording unavailable"
            )
        sid, item = eligible[0]
        # Revalidate the actual archive before deleting; never prune raw/failed sessions.
        metadata = json.loads(item["sidecar"].read_text(encoding="utf8"))
        if (
            metadata.get("zip_mtime_ns") is not None
            and item["zip"].stat().st_mtime_ns != metadata["zip_mtime_ns"]
        ):
            raise RuntimeError("old archive changed after verification; protected from pruning")
        item["zip"].unlink()
        item["sidecar"].unlink()
        sync_directory(root)
        removed.append(sid)
        entries.pop(sid)
        with (root / "retention.jsonl").open("a", encoding="utf8") as f:
            f.write(
                json.dumps(
                    dict(at_unix_ns=time.time_ns(), removed_session=sid, reason="retention_budget")
                )
                + "\n"
            )
    return removed


def archive(root, folder, uid, level=1):
    root = Path(root).resolve()
    folder = Path(folder)
    if (
        folder.is_symlink()
        or folder.parent.resolve() != root
        or not folder.name.startswith("flight-")
        or not folder.name.endswith(".open")
    ):
        raise ValueError("not an owned session directory")
    sid = folder.name[:-5]
    target = root / (sid + ".zip")
    sidecar = root / (sid + ".verified.json")
    temporary = root / (sid + ".zip.tmp")
    if any(p.is_symlink() for p in (target, sidecar, temporary)):
        raise ValueError("archive path must not be a symlink")
    # A crash between ZIP commit and raw cleanup is recoverable, but source changes
    # after commit must never be discarded by the fast recovery path.
    if target.exists() and sidecar.exists():
        old = json.loads(sidecar.read_text(encoding="utf8"))
        if not (
            old.get("verified") is True
            and old.get("uid") == uid
            and old.get("session_id") == sid
            and sha(target) == old.get("zip_sha256")
        ):
            raise RuntimeError("existing archive not verified; raw remains protected")
        # Cleanup may previously have stopped after removing session.json. Recover
        # only if every remaining file is an unchanged member of the verified ZIP.
        with zipfile.ZipFile(target) as z:
            saved = json.loads(z.read("session.json"))
            recorded = json.loads(z.read("ARCHIVE-MANIFEST.json"))
            if (
                saved.get("schema") != SCHEMA
                or saved.get("uid") != uid
                or saved.get("session_id") != sid
                or not saved.get("closed")
                or recorded.get("session_id") != sid
            ):
                raise ValueError("verified archive ownership mismatch")
            members = recorded["files"]
            for base, dirs, names in os.walk(folder, followlinks=False):
                if any((Path(base) / d).is_symlink() for d in dirs):
                    raise ValueError("symlink in cleanup source")
                for name in names:
                    p = Path(base) / name
                    rel = p.relative_to(folder).as_posix()
                    expected = members.get(rel)
                    if (
                        p.is_symlink()
                        or not stat.S_ISREG(p.stat().st_mode)
                        or not expected
                        or p.stat().st_size != expected["bytes"]
                        or sha(p) != expected["sha256"]
                    ):
                        raise RuntimeError(
                            "source changed after archive verification; source retained"
                        )
        shutil.rmtree(folder)
        sync_directory(root)
        return old
    meta = owned_directory(root, folder, uid)
    if not meta.get("closed"):
        raise ValueError("active recording cannot be archived")
    committed_without_sidecar = target.exists()
    if committed_without_sidecar:
        temporary = target

    def source_files():
        files = []
        for base, dirs, names in os.walk(folder, followlinks=False):
            for d in dirs:
                if (Path(base) / d).is_symlink():
                    raise ValueError("symlink in recording directory")
            for name in names:
                p = Path(base) / name
                if p.is_symlink() or not stat.S_ISREG(p.stat().st_mode):
                    raise ValueError("unsafe recording member")
                files.append(p)
        return files

    files = source_files()
    hashes = {
        p.relative_to(folder).as_posix(): dict(bytes=p.stat().st_size, sha256=sha(p)) for p in files
    }
    if "ARCHIVE-MANIFEST.json" in hashes:
        raise ValueError("reserved archive member name")
    if not committed_without_sidecar:
        with zipfile.ZipFile(
            temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=level, allowZip64=True
        ) as z:
            for p in files:
                z.write(p, p.relative_to(folder).as_posix())
            z.writestr(
                "ARCHIVE-MANIFEST.json",
                json.dumps(
                    dict(schema="argus.flight.members.v1", session_id=sid, files=hashes),
                    ensure_ascii=False,
                    indent=2,
                ),
            )
    with temporary.open("r+b") as f:
        os.fsync(f.fileno())
    with zipfile.ZipFile(temporary) as z:
        bad = z.testzip()
        if bad:
            raise RuntimeError("ZIP CRC verification failed: " + bad)
        if set(z.namelist()) != set(hashes) | {"ARCHIVE-MANIFEST.json"}:
            raise RuntimeError("ZIP file set mismatch")
        recorded = json.loads(z.read("ARCHIVE-MANIFEST.json"))
        if recorded.get("files") != hashes or recorded.get("session_id") != sid:
            raise RuntimeError("ZIP manifest mismatch")
        for name, expected in hashes.items():
            digest = hashlib.sha256()
            count = 0
            with z.open(name) as f:
                while True:
                    chunk = f.read(1024 * 1024)
                    if not chunk:
                        break
                    digest.update(chunk)
                    count += len(chunk)
            if count != expected["bytes"] or digest.hexdigest() != expected["sha256"]:
                raise RuntimeError("ZIP member SHA verification failed")
    if {p.relative_to(folder).as_posix() for p in source_files()} != set(hashes):
        raise RuntimeError("session member set changed while archiving")
    for p in files:
        expected = hashes[p.relative_to(folder).as_posix()]
        if p.stat().st_size != expected["bytes"] or sha(p) != expected["sha256"]:
            raise RuntimeError("session changed while archiving")
    if not committed_without_sidecar:
        os.replace(temporary, target)
        sync_directory(root)
    result = dict(
        schema="argus.flight.archive.v1",
        uid=uid,
        session_id=sid,
        verified=True,
        created_unix_ns=meta["created_unix_ns"],
        verified_unix_ns=time.time_ns(),
        zip_bytes=target.stat().st_size,
        zip_mtime_ns=target.stat().st_mtime_ns,
        zip_sha256=sha(target),
        members=len(hashes),
        ended_reason=meta.get("ended_reason"),
        incomplete=meta.get("incomplete", False),
        flight_observed=meta.get("flight_observed", False),
        evidence_counts=meta.get("counts", {}),
        recording_errors=meta.get("errors", []),
        evidence_complete=meta.get("evidence_complete", False),
        evidence_completeness=meta.get(
            "evidence_completeness", {"missing": ["legacy session has no coverage declaration"]}
        ),
    )
    atomic_json(sidecar, result)
    # Delete only the owned, closed source directory whose member hashes were just rechecked.
    owned_directory(root, folder, uid)
    shutil.rmtree(folder)
    sync_directory(root)
    return result


class Session:
    def __init__(self, root, uid, binding, reason, retain_count=10):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        retain(self.root, uid, retain_count, reserve=1)
        created = time.time_ns()
        sid = "flight-" + str(created) + "-" + uuid.uuid4().hex[:8]
        self.folder = self.root / (sid + ".open")
        self.folder.mkdir()
        self.meta = dict(
            schema=SCHEMA,
            session_id=sid,
            uid=uid,
            binding=list(binding),
            created_unix_ns=created,
            started_monotonic_ns=time.monotonic_ns(),
            start_reason=reason,
            closed=False,
            flight_observed=False,
            counts={},
            errors=[],
        )
        self.streams = {}
        self.raw_bytes = 0
        self.raw_disabled = False
        self.raw_keys = set()
        self.cmc_keys = set()
        self.follow_decision_records = 0
        self.source_cutoff_ns = None
        self.save_meta()

    def save_meta(self):
        atomic_json(self.folder / "session.json", self.meta)

    def line(self, name, value):
        if name not in self.streams:
            self.streams[name] = (self.folder / name).open("a", encoding="utf8")
        self.streams[name].write(
            json.dumps(sanitize(value), ensure_ascii=False, allow_nan=False) + "\n"
        )
        self.meta["counts"][name] = self.meta["counts"].get(name, 0) + 1
        if name == "cmc.jsonl":
            pixels = (value.get("record") or {}).get("pixels") or {}
            if pixels.get("source") is not None:
                self.cmc_keys.add(
                    (
                        pixels.get("source"),
                        pixels.get("frame_id"),
                        pixels.get("captured_monotonic_ns"),
                    )
                )
        if (
            name == "follow_diagnostics.jsonl"
            and (value.get("record") or {}).get("schema") == "argus.follow.decision.v1"
        ):
            self.follow_decision_records += 1

    def pixels(self, meta, raw, model, nms):
        record = dict(meta)
        header = meta.get("header") or {}
        sample = header.get("received_monotonic_ns")
        if (
            self.source_cutoff_ns is not None
            and type(sample) is int
            and sample > self.source_cutoff_ns
        ):
            self.meta["counts"]["raw_frames_ignored_after_source_cutoff"] = (
                self.meta["counts"].get("raw_frames_ignored_after_source_cutoff", 0) + 1
            )
            return False
        self.raw_keys.add(
            (meta.get("source"), header.get("frame_id"), header.get("received_monotonic_ns"))
        )
        for name, blob in [
            ("raw_ir.bin", raw),
            ("model_input.bin", model),
            ("nms_output.bin", nms),
        ]:
            if name not in self.streams:
                self.streams[name] = (self.folder / name).open("ab")
            f = self.streams[name]
            record[name] = dict(
                offset=f.tell(), bytes=len(blob), sha256=hashlib.sha256(blob).hexdigest()
            )
            f.write(blob)
            self.raw_bytes += len(blob)
        self.line("frames.jsonl", record)
        return True

    def freeze_sources(self, cutoff_ns):
        if self.source_cutoff_ns is not None:
            return
        existing = [key[2] for key in self.raw_keys if type(key[2]) is int]
        self.source_cutoff_ns = max([int(cutoff_ns)] + existing)
        self.meta["evidence_source_cutoff_monotonic_ns"] = self.source_cutoff_ns
        self.meta["evidence_source_cutoff_reason"] = (
            "confirmed actual landed and disarmed; remaining postflight time drains delayed CMC/follow records"
        )
        self.save_meta()

    def flush(self):
        for f in self.streams.values():
            f.flush()

    def close(self, reason, incomplete=False):
        failures = []
        for f in self.streams.values():
            try:
                f.flush()
                os.fsync(f.fileno())
            except OSError as exc:
                failures.append("source flush: " + str(exc))
            finally:
                f.close()
        self.streams.clear()
        if failures:
            self.meta["errors"].extend(failures)
            incomplete = True
        missing = []
        if not self.raw_keys:
            missing.append("original_IR_frames")
        if not self.cmc_keys:
            missing.append("CMC_records")
        if not self.follow_decision_records:
            missing.append("follow_decisions_not_recorded_or_no_active_follow")
        envelopes = {}
        for source, _, sample in self.raw_keys:
            if type(sample) is not int:
                continue
            if source not in envelopes:
                envelopes[source] = [sample, sample]
            else:
                envelopes[source] = [
                    min(envelopes[source][0], sample),
                    max(envelopes[source][1], sample),
                ]
        in_envelope = {
            key
            for key in self.cmc_keys
            if key[0] in envelopes
            and type(key[2]) is int
            and envelopes[key[0]][0] <= key[2] <= envelopes[key[0]][1]
        }
        coverage = dict(
            raw_frame_keys=len(self.raw_keys),
            cmc_frame_keys=len(self.cmc_keys),
            cmc_frame_keys_in_raw_envelope=len(in_envelope),
            cmc_frame_keys_outside_raw_envelope=len(self.cmc_keys - in_envelope),
            raw_sample_envelopes_ns=envelopes,
            raw_without_cmc=len(self.raw_keys - self.cmc_keys),
            cmc_without_raw=len(in_envelope - self.raw_keys),
            follow_decision_records=self.follow_decision_records,
        )
        if coverage["raw_without_cmc"] or coverage["cmc_without_raw"]:
            missing.append("raw_CMC_frame_key_coverage")
        complete = not missing and not failures and not self.meta["errors"]
        for item in missing:
            self.meta["errors"].append("evidence_missing: " + item)
        self.meta.update(
            closed=True,
            closed_unix_ns=time.time_ns(),
            closed_monotonic_ns=time.monotonic_ns(),
            ended_reason=reason,
            incomplete=incomplete,
            raw_bytes=self.raw_bytes,
            coverage=coverage,
            evidence_complete=complete,
            evidence_completeness=dict(
                coverage,
                missing=missing,
                note="ZIP integrity verification is independent from evidence completeness; camera frames skipped before inference are not recoverable",
            ),
        )
        self.save_meta()
        return self.folder
