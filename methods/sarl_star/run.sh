#!/usr/bin/env bash
# Run SARL* against a RobotSNAP build.
#
# SARL* is a pretrained social-attention network that its authors shipped as a
# ROS 1 image. The image is used as it is - nothing inside it is patched - and
# the things it cannot know are adapted around it by tools/ros1/:
#
#   tools/ros1/compose.yaml               a ROS 1 master, ros1_bridge and the
#                                         ROS 1 view of RobotSNAP, brought up
#                                         together, the same for every method
#   tools/ros1/robotsnap_ros1_adapter.py  /odom -> /robot_odom, and the JSON
#                                         crowd -> /social_sim/agents
#
# One tmux session, one window per responsibility:
#   unity    - the RobotSNAP player alone, without the bridge
#   bridge   - "python -m robotsnap bridge --ros2"
#   method   - the whole ROS 1 side, from lib/common.sh
#   control  - hands the robot to ROS, so /cmd_vel is what drives it
#   rviz     - the shared view, tools/scene.rviz
#   goal     - a shell ready to run tools/send_goal.py
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=sarl_star
SESSION=sarl_star
ROBOT=robot_1

IMAGE="$(method_env_value "${NAME}" IMAGE)"
METHOD_ROS_ENV="$(method_env_value "${NAME}" METHOD_ROS_ENV)"
METHOD_LAUNCH="$(method_env_value "${NAME}" METHOD_LAUNCH)"
GOAL_HINT="$(method_env_value "${NAME}" GOAL_HINT)"
RVIZ_CONFIG="$(baseline_root)/tools/scene.rviz"

# The command that brings the ROS 1 side up, asked of lib/common.sh so the pane
# and --print-method can only ever show the same string.
ros1_cmd() {
    ros1_up_command "${NAME}" "${IMAGE}" "${METHOD_LAUNCH}" "${METHOD_ROS_ENV}" "${ROBOT}"
}

usage() {
    cat <<EOF
Usage: run.sh [--headless] [--kill] [--stop] [--robot NAME] [--print-method] [--help]

Start SARL* and RobotSNAP in a tmux session named '${SESSION}'.

  --headless        run the RobotSNAP app with -batchmode -nographics
  --robot NAME      robot the goal and SARL* drive (default: ${ROBOT})
  --kill            kill an existing '${SESSION}' session and its ROS 1 containers
  --stop            stop the '${SESSION}' session and its ROS 1 containers
  --print-method    print the ROS 1 launch command and exit
  -h, --help        show this help

Windows: unity bridge method control rviz(2) goal(2).
In the 'goal' window: ${GOAL_HINT}
EOF
}

KILL=0
HEADLESS=0
STOP=0
PRINT_METHOD=0
while (($#)); do
    case "$1" in
        -h|--help)      usage; exit 0 ;;
        --kill)         KILL=1 ;;
        --stop)         STOP=1 ;;
        --headless)     HEADLESS=1 ;;
        --print-method) PRINT_METHOD=1 ;;
        --robot)        ROBOT="${2:?--robot needs a name}"; shift ;;
        *) echo "run.sh: unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if ((PRINT_METHOD)); then
    ros1_cmd; echo; exit 0
fi

# Killing the tmux session takes the panes with it, but the docker *client* in
# the method pane dying does not stop the containers it started: without the
# compose down the four services outlive the window that launched them, and the
# next run finds the ports taken.
stop_all() {
    tmux kill-session -t "${SESSION}" 2>/dev/null \
        && echo "run.sh: stopped the '${SESSION}' session"
    # ros1_down_command sets the inert values docker compose needs to
    # interpolate tools/ros1/compose.yaml during a ``down``; without them the
    # stop fails and the four services survive.
    eval "$(ros1_down_command "${NAME}")" || true
    echo "run.sh: brought the ROS 1 containers down"
    return 0
}

if ((STOP)); then
    stop_all
    exit 0
fi

require_command tmux
require_command docker
[ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ] \
    || die "run.sh: ROS ${ROS_DISTRO:-humble} is not installed; expected /opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    if ((KILL)); then
        stop_all
    else
        die "run.sh: tmux session '${SESSION}' already exists; re-run with --kill to replace it"
    fi
fi

mapfile -t METHOD_SOURCES < <(method_sources "${NAME}")
ENV_PREFIX="$(ros_env "${HERE}" "${METHOD_SOURCES[@]}")"

if ((HEADLESS)); then
    UNITY_FLAGS="-batchmode -nographics"
else
    UNITY_FLAGS=""
fi

# -- unity: the player alone, unless it is already open ------------------------
session_create "${SESSION}" "${HERE}"
window_unity_guard "${SESSION}" "${HERE}" "${UNITY_FLAGS}"

# -- bridge: RobotSNAP served to the ROS 2 graph -------------------------------
window_bridge "${SESSION}" "${HERE}" "${ENV_PREFIX}"

# -- method: the whole ROS 1 side, from lib/common.sh --------------------------
window_new "${SESSION}" method "${HERE}"
window_run "${SESSION}" method "$(ros1_cmd)"

# -- control: hand the robot to ROS --------------------------------------------
window_control "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${ROBOT}"

# -- rviz ---------------------------------------------------------------------
window_rviz "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${RVIZ_CONFIG}" "${ROBOT}"

# -- goal ---------------------------------------------------------------------
window_goal "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${GOAL_HINT}" "${NAME}" "${ROBOT}"

session_close "${SESSION}" "robotsnap-baseline stop ${NAME}" \
    "unity bridge method control rviz(2) goal(2)"
