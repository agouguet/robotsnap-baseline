#!/usr/bin/env bash
#
# Verify SARL* without starting a simulation.
#
# It answers the questions a launch would otherwise answer with a crash: is the
# image here, does it carry the node, the helpers, the built workspace and the
# pretrained checkpoint, is the ROS 1 side of the workspace complete, and is the
# ROS 1 session free.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=sarl_star
IMAGE="$(method_env_value "${NAME}" IMAGE)"

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

# The node, the helpers, the built workspace and the pretrained checkpoint all
# live inside the image; a launch fails on the first of them that is missing, so
# check them here instead.
if docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    docker run --rm --entrypoint bash "${IMAGE}" -lc \
        'test -f /root/sarl_ws/src/sarl_star/sarl_star_ros/scripts/sarl_star_node.py &&
         test -f /root/sarl_ws/src/sarl_star/sarl_star_ros/scripts/agents_to_people.py &&
         test -d /root/sarl_ws/devel &&
         test -f /root/sarl_ws/src/sarl_star/sarl_star_ros/CrowdNav/crowd_nav/data/output/rl_model.pth &&
         test -f /root/sarl_ws/src/sarl_star/sarl_star_ros/CrowdNav/crowd_nav/data/output/env.config &&
         test -f /root/sarl_ws/src/sarl_star/sarl_star_ros/CrowdNav/crowd_nav/data/output/policy.config' \
        >/dev/null 2>&1
    expect $? "the image carries the node, the helpers, the built workspace and the checkpoint"

    docker run --rm --entrypoint bash "${IMAGE}" -lc \
        'test -f /opt/ros/kinetic/setup.bash' >/dev/null 2>&1
    expect $? "the image carries ROS 1 Kinetic"
fi

[ -n "$(method_env_value "${NAME}" METHOD_LAUNCH)" ]
expect $? "the manifest declares a launch command"
[ -n "$(method_env_value "${NAME}" METHOD_ROS_ENV)" ]
expect $? "the manifest declares a ROS 1 environment"

# The ROS 1 session runs under its own compose project. A project already up
# would collide with the next launch, so its absence is part of being ready.
running="$(docker ps \
    --filter "label=com.docker.compose.project=$(ros1_project "${NAME}")" \
    --format '{{.Names}}' 2>/dev/null)"
if [ -n "${running}" ]; then
    fail "a SARL* ROS 1 session is already running: $(printf '%s' "${running}" | tr '\n' ' ')"
    FAILED=1
else
    ok "no SARL* ROS 1 session is running"
fi

if [ -x "$(venv_python)" ]; then
    ( source_ros >/dev/null 2>&1; "$(venv_python)" -c "import rclpy" 2>/dev/null )
    expect $? "the workspace interpreter carries rclpy"
else
    fail "no interpreter at $(venv_python); set ROBOTSNAP_WS or build the workspace"
    FAILED=1
fi

for tool in \
    tools/ros1/compose.yaml \
    tools/ros1/robotsnap_ros1_adapter.py \
    tools/send_goal.py \
    tools/set_ros_control.py \
    tools/scene.rviz
do
    [ -f "$(baseline_root)/${tool}" ]
    expect $? "shared tool is present: ${tool}"
done

if [ "${FAILED}" -eq 0 ]; then
    printf '\n%s is ready: run it with "robotsnap-baseline run %s"\n' "${NAME}" "${NAME}"
else
    printf '\n%s is not ready; run "robotsnap-baseline install %s" first\n' "${NAME}" "${NAME}" >&2
fi
exit "${FAILED}"
