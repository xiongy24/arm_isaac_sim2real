#!/bin/bash
export EXP_PATH=/home/xy/.local/share/ov/pkg/isaac-sim-4.1.0/exts
export ISAAC_PATH=/home/xy/.local/share/ov/pkg/isaac-sim-4.1.0
export CARB_APP_PATH=${ISAAC_PATH}
export ISAAC_PYTHON=${ISAAC_PATH}/python.sh
export PYTHONPATH=${ISAAC_PATH}/pip_prebundle:${PYTHONPATH}
export PYTHONPATH=${ISAAC_PATH}/exts:${PYTHONPATH}