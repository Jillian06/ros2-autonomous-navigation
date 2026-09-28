#!/usr/bin/env bash
set -euo pipefail
PYTHONPATH=autonomous_navigation pytest -q autonomous_navigation/test/test_planning.py autonomous_navigation/test/test_control.py
