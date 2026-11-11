#!/usr/bin/env bash
# Wrapper: sources the workspace overlay then runs the Python smoke test.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
# ROS setup scripts reference unset vars
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROOT}/install/setup.bash"
set -u

exec python3 "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_smoke.py"
