"""CLI compatibility only: replace this process with the canonical C++ executable."""
import os
from pathlib import Path
import sys


def main():
    root = Path(__file__).resolve().parents[3]
    executable = root / "build/bin/argus-onboard"
    if not executable.is_file():
        raise SystemExit("C++ executable missing; run bash scripts/build.sh. Python orchestration fallback is prohibited.")
    os.execv(str(executable), [str(executable), *sys.argv[1:]])
