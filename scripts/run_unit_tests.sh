#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PYTHONPATH=lsy_autonomous_navigation pytest -q \
  lsy_autonomous_navigation/test/test_planning.py \
  lsy_autonomous_navigation/test/test_control.py
