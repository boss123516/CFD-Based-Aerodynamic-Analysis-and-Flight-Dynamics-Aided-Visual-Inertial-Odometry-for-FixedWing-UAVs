#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
unset PYTHONPATH
mkdir -p runs
MAP_SERVER_PID=""
MAP_PORT=8765
SERVER_ARGS=()
BROWSER_ARGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --aircraft-profile) SERVER_ARGS+=("$1" "$2"); shift 2 ;;
    --port) MAP_PORT="$2"; SERVER_ARGS+=("$1" "$2"); shift 2 ;;
    *) BROWSER_ARGS+=("$1"); shift ;;
  esac
done
cleanup() { if [ -n "$MAP_SERVER_PID" ]; then kill -TERM "$MAP_SERVER_PID" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
if ! .venv/bin/python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:$MAP_PORT/api/config',timeout=2)" 2>/dev/null; then
  .venv/bin/python -u scripts/serve_map.py "${SERVER_ARGS[@]}" > runs/map_server.log 2>&1 &
  MAP_SERVER_PID=$!
  for attempt in {1..30}; do
    if .venv/bin/python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:$MAP_PORT/api/config',timeout=1)" 2>/dev/null; then break; fi
    sleep 0.2
  done
fi
if [ -z "$MAP_SERVER_PID" ] && [ "${#SERVER_ARGS[@]}" -gt 0 ]; then
  echo 'A server already uses this port. Choose another --port to apply a new aircraft profile.' >&2
  exit 1
fi
.venv/bin/python -u scripts/open_map.py --url "http://127.0.0.1:$MAP_PORT" "${BROWSER_ARGS[@]}"
