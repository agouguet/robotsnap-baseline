#!/usr/bin/env bash
#
# Verify CoHAN-Nav2 without starting a simulation.
#
# Answers one question - "would a launch find what it needs" - by reading the
# workspace the launch would source. It never starts nav2. Exit code is 0 when
# every check passes, 1 otherwise.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=cohan_nav2
COHAN_WS="${COHAN_WS:-$(baseline_root)/$(method_env_value "${NAME}" WORKSPACE)}"
SETUP="${COHAN_WS}/install/setup.bash"
PARAMS="${HERE}/cohan_robotsnap_params.yaml"
APT="$(method_env_value "${NAME}" APT)"

FAILED=0
expect() {
    if [ "$1" = "0" ]; then
        ok "$2"
    else
        fail "$2"
        FAILED=1
    fi
}

[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; expect $? "ROS ${ROS_DISTRO} is installed"
[ -f "${SETUP}" ]; expect $? "workspace is built: ${COHAN_WS}"

if [ -f "${SETUP}" ]; then
    local_head="$(git -C "${COHAN_WS}" rev-parse HEAD 2>/dev/null || echo none)"
    [ "${local_head}" = "${COHAN_COMMIT}" ]
    expect $? "workspace is at the pinned commit ${COHAN_COMMIT} (found ${local_head})"

    for local_pkg in cohan_sim_navigation agent_path_prediction invisible_humans_detection cohan_layers; do
        [ -d "${COHAN_WS}/install/${local_pkg}" ]
        expect $? "package built: ${local_pkg}"
    done
fi

dpkg -s "${APT}" >/dev/null 2>&1; expect $? "${APT} is installed (the HATEB plugin links against it)"

[ -f "${PARAMS}" ]; expect $? "parameters file is present: ${PARAMS}"
if [ -f "${PARAMS}" ]; then
    # `base_link`, not `chassis_link`. The stacks hardcode `base_link` in C++
    # (invisible_humans_detection's FOOTPRINT_FRAME, HATEB's own wrapper and
    # visualisation), and the robot's own frame is not one name: the Jackal
    # publishes its odometry under `chassis_link`, the other prefabs under
    # `base_link`. The parameters therefore name the one name every stack
    # agrees on, and tools/frames_alias.py hangs it off whatever body frame
    # `/odom` announces. Naming `chassis_link` here was the old assumption that
    # the scenario always loads a Jackal.
    grep -qE 'robot_base_frame:[[:space:]]*base_link' "${PARAMS}"
    expect $? "parameters name the robot frame every stack hardcodes (base_link)"
    [ -f "$(baseline_root)/tools/frames_alias.py" ]
    expect $? "the shared frame alias that carries the other frame names is present"
    grep -qE 'max_vel_y:[[:space:]]*0' "${PARAMS}"
    expect $? "parameters keep the robot non-holonomic (max_vel_y: 0)"
fi

if [ "${FAILED}" -eq 0 ]; then
    printf '\n%s is ready: run it with "robotsnap-baseline run %s"\n' "${NAME}" "${NAME}"
else
    printf '\n%s is not ready; run "robotsnap-baseline install %s" first\n' "${NAME}" "${NAME}" >&2
fi
exit "${FAILED}"
