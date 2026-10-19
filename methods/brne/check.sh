#!/usr/bin/env bash
#
# Verify BRNE without starting a simulation.
#
# It answers the questions a launch would otherwise answer with a crash: is the
# image here, can the adapters import what they need, is the container free.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=brne
IMAGE="$(method_env_value "${NAME}" IMAGE)"
CONTAINER="$(method_env_value "${NAME}" CONTAINER)"
SOURCES="$(method_env_value "${NAME}" SOURCES)"

FAILED=0
expect() {
    if [ "$1" = "0" ]; then
        ok "$2"
    else
        fail "$2"
        FAILED=1
    fi
}

command -v docker >/dev/null 2>&1; expect $? "docker is installed"
docker image inspect "${IMAGE}" >/dev/null 2>&1; expect $? "image is present: ${IMAGE}"

image_arch="$(docker image inspect -f '{{.Architecture}}' "${IMAGE}" 2>/dev/null)"
[ "${image_arch}" = "$(uname -m | sed 's/x86_64/amd64/')" ]
expect $? "image architecture matches this host (${image_arch:-unknown})"

[ -f "$(workspace_root)/${SOURCES}" ]; expect $? "ROS2 workspace setup is present: ${SOURCES}"

if [ ! -x "$(venv_python)" ]; then
    fail "no interpreter at $(venv_python); set ROBOTSNAP_WS or build the workspace"
    exit 1
fi
(
    source_ros >/dev/null 2>&1
    source_setup "$(workspace_root)/${SOURCES}" >/dev/null 2>&1
    "$(venv_python)" -c "import agents_msgs, rclpy" 2>/dev/null
)
expect $? "the adapters can import agents_msgs and rclpy"

if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    fail "a container named '${CONTAINER}' is already running"
else
    ok "no '${CONTAINER}' container is running"
fi

if [ "${FAILED}" -eq 0 ]; then
    printf '\n%s is ready: run it with "robotsnap-baseline run %s"\n' "${NAME}" "${NAME}"
else
    printf '\n%s is not ready; run "robotsnap-baseline install %s" first\n' "${NAME}" "${NAME}" >&2
fi
exit "${FAILED}"
