#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
unset PYTHONPATH
mkdir -p runs
MAP_SERVER_PID=""
cleanup() { if [ -n "$MAP_SERVER_PID" ]; then kill -TERM "$MAP_SERVER_PID" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
if ! .venv/bin/python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8765/api/config',timeout=2)" 2>/dev/null; then
  .venv/bin/python -u scripts/serve_map.py > runs/map_server.log 2>&1 &
  MAP_SERVER_PID=$!
  for attempt in {1..30}; do
    if .venv/bin/python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8765/api/config',timeout=1)" 2>/dev/null; then break; fi
    sleep 0.2
  done
fi
.venv/bin/python -u scripts/open_map.py "$@"
