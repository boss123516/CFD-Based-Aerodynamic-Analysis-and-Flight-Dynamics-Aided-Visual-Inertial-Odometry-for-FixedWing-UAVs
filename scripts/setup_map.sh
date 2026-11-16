#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
unset PYTHONPATH
test -x .venv/bin/python || python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.lock.txt -r requirements-renderer.lock.txt
mkdir -p external/cesium
if [ ! -f external/cesium/Build/Cesium/Cesium.js ]; then
  npm pack cesium@1.124.0 --pack-destination external --silent
  tar -xzf external/cesium-1.124.0.tgz -C external/cesium --strip-components=1 package/Build package/LICENSE.md
fi
echo 'Ready: ./run.sh map --record'
