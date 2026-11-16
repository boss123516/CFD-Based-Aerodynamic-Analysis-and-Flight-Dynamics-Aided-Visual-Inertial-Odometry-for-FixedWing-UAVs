#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"
export PYTHONNOUSERSITE=1
unset PYTHONPATH
case "${1:-help}" in
  generate) shift; exec .venv/bin/python scripts/generate_flight.py "$@" ;;
  validate) shift; exec .venv/bin/python scripts/validate_dataset.py "$@" ;;
  plot) shift; exec .venv/bin/python scripts/plot_flight.py "$@" ;;
  render) shift; exec .venv/bin/python scripts/render_airsim.py "$@" ;;
  pipeline) shift; exec .venv/bin/python scripts/pipeline.py "$@" ;;
  map) shift; exec bash scripts/run_map.sh "$@" ;;
  test) shift; .venv/bin/python scripts/test_sensor_contract.py; exec .venv/bin/python -m unittest discover -s tests -v "$@" ;;
  *) echo 'Usage: ./run.sh {generate|render|validate|plot|pipeline {demo|run|evaluate}|map [--record]|test} [options]' ;;
esac
