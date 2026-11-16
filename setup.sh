#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"
unset PYTHONPATH
export PYTHONNOUSERSITE=1
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.lock.txt
./run.sh test
