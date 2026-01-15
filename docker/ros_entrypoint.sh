#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /workspace_ur5/install/setup.bash ]; then
  source /workspace_ur5/install/setup.bash
fi

exec "$@"
