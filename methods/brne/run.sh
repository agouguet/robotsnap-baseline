#!/usr/bin/env bash
# Run BRNE against a RobotSNAP build.
#
# BRNE is a pretrained crowd-navigation policy that its authors shipped as a
# Docker image. The image is used as it is - nothing inside it is patched - and
# the two things it cannot know are adapted beside it:
#
#   tools/odom_relay.py     /odom        -> /robot_odom        (same message)
#   tools/agents_adapter.py /simulation/agents -> /social_sim/agents, world frame
#
# One tmux session, one window per responsibility:
#   unity    - the RobotSNAP player alone, without the bridge
#   bridge   - "python -m robotsnap bridge --ros2"
#   method   - the BRNE container, and the two adapters that feed it
#   control  - hands the robot to ROS, so /cmd_vel is what drives it
#   rviz     - the shared view, tools/scene.rviz
#   goal     - a shell ready to run tools/send_goal.py
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=brne
SESSION=brne
ROBOT=robot_1

IMAGE="$(method_env_value "${NAME}" IMAGE)"
CONTAINER="$(method_env_value "${NAME}" CONTAINER)"
GOAL_HINT="$(method_env_value "${NAME}" GOAL_HINT)"
RVIZ_CONFIG="$(baseline_root)/tools/scene.rviz"

# BRNE's entry point, exactly as its image documents it. Kept as one string so
# `run.sh --print-method` and the tmux pane can only ever show the same command.
container_cmd() {
    printf 'docker run --rm --name %s --network host -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 --entrypoint "" %s bash -c %s' \
        "${CONTAINER}" "${IMAGE}" \
        "'sleep 1 && source /opt/ros/humble/setup.bash && source /root/brne_ws/install/setup.bash && stdbuf -o L ros2 launch crowd_nav algorithm.launch.xml'"
}

usage() {
    cat <<EOF
Usage: run.sh [--headless] [--kill] [--stop] [--print-method] [--help]

Start BRNE and RobotSNAP in a tmux session named '${SESSION}'.

  --headless        run the RobotSNAP app with -batchmode -nographics
  --kill            kill an existing '${SESSION}' session and its container
  --stop            stop the '${SESSION}' session and its container
  --print-method    print the container command and exit
  -h, --help        show this help

Windows: unity bridge method control rviz(2) goal(2).
In the 'goal' window: ${GOAL_HINT}
EOF
}

KILL=0
HEADLESS=0
STOP=0
while (($#)); do
    case "$1" in
        -h|--help)      usage; exit 0 ;;
        --kill)         KILL=1 ;;
        --stop)         STOP=1 ;;
        --headless)     HEADLESS=1 ;;
        --print-method) container_cmd; echo; exit 0 ;;
        *) echo "run.sh: unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

# Killing the session takes the panes with it, but the docker *client* dying
# does not stop the container it started: the method would outlive the window
# that launched it, and the next run would find the name taken.
stop_everything() {
    tmux kill-session -t "${SESSION}" 2>/dev/null \
        && echo "run.sh: stopped the '${SESSION}' session"
    docker stop "${CONTAINER}" >/dev/null 2>&1 \
        && echo "run.sh: stopped the '${CONTAINER}' container"
    return 0
}

if ((STOP)); then
    stop_everything
    exit 0
fi

require_command tmux
require_command docker
[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ] \
    || die "run.sh: ROS ${ROS_DISTRO} is not installed; expected /opt/ros/${ROS_DISTRO}/setup.bash"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    if ((KILL)); then
        stop_everything
    else
        die "run.sh: tmux session '${SESSION}' already exists; re-run with --kill to replace it"
    fi
fi
if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    die "run.sh: a container named '${CONTAINER}' is already running; stop it first"
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

# -- method: the container, and the adapters that make it see RobotSNAP ---------
window_new "${SESSION}" method "${HERE}"
window_run "${SESSION}" method "$(container_cmd)"
pane_split "${SESSION}:method" -h "${HERE}" \
    "${ENV_PREFIX} && exec $(venv_python) $(baseline_root)/tools/odom_relay.py --alias-frame brne_odom"
pane_split "${SESSION}:method" -v "${HERE}" \
    "${ENV_PREFIX} && exec $(venv_python) $(baseline_root)/tools/agents_adapter.py"
tmux select-layout -t "${SESSION}:method" tiled

# -- control: hand the robot to ROS --------------------------------------------
window_control "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${ROBOT}"

# -- rviz ---------------------------------------------------------------------
window_rviz "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${RVIZ_CONFIG}" "${ROBOT}"

# -- goal ---------------------------------------------------------------------
window_goal "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${GOAL_HINT}" "${NAME}" "${ROBOT}"

session_close "${SESSION}" "robotsnap-baseline stop ${NAME}" \
    "unity bridge method(3) control rviz(2) goal(2)"
