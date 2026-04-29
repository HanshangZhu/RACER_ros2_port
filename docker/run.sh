#!/usr/bin/env bash
# Run the RACER container with X11 passthrough for Rviz.
# Repo is bind-mounted at /catkin_ws/src/RACER. build/ and devel/ persist on host.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CACHE_DIR="${REPO_ROOT}/docker/.cache"
mkdir -p "${CACHE_DIR}/build" "${CACHE_DIR}/devel" "${CACHE_DIR}/logs"

# Allow local root (container) to use the host X server
xhost +local:root >/dev/null 2>&1 || true

exec docker run --rm -it \
    --name racer \
    --net=host \
    --ipc=host \
    -e DISPLAY="${DISPLAY:-:1}" \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${HOME}/.Xauthority:/tmp/.docker.xauth:ro" \
    -v "${REPO_ROOT}:/catkin_ws/src/RACER:rw" \
    -v "${CACHE_DIR}/build:/catkin_ws/build:rw" \
    -v "${CACHE_DIR}/devel:/catkin_ws/devel:rw" \
    -v "${CACHE_DIR}/logs:/root/.ros/log:rw" \
    racer:noetic "$@"
