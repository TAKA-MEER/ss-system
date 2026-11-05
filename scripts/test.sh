#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

cd /ws/ss_ws
colcon build

set +u
source install/setup.bash
set -u

cd /ws
python3 -m pytest ss_ws/src/ss_locator/tests
