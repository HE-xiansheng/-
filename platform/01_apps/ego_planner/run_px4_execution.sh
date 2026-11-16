#!/usr/bin/env bash
set -eo pipefail
PLANNER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$PLANNER_DIR/../../env.sh"
export PYTHONNOUSERSITE=1
exec /usr/bin/python3 -B "$PLANNER_DIR/px4_execution/manage.py" "${1:-status}"
