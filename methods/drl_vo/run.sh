#!/usr/bin/env bash
# Run DRL-VO against a RobotSNAP build.
#
# DRL-VO's published code is ROS 1. This integration takes the other road: the
# policy and the observation builder are the same maths and the same checkpoint,
# rewired as ROS 2 nodes beside this script, so the robot is driven over
# "robotsnap bridge --ros2" with no container and no bridge in between.
#
# One tmux session, one window per responsibility:
#   unity    - the RobotSNAP player alone, without the bridge
#   bridge   - "python -m robotsnap bridge --ros2"
#   drl_vo   - the observation builder and the policy
#   control  - hands the robot to ROS, so /cmd_vel is what drives it
#   rviz     - rviz2, on the session's own graph
#   goal     - a shell ready to run send_goal.py
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=drl_vo
SESSION=drl_vo
ROBOT=robot_1

MODEL="${HERE}/model/drl_vo.zip"
RVIZ_CONFIG="$(baseline_root)/tools/scene.rviz"
GOAL_HINT="$(method_env_value "${NAME}" GOAL_HINT)"

usage() {
    cat <<EOF
Usage: run.sh [--headless] [--kill] [--stop] [--robot NAME] [--help]

Start DRL-VO and RobotSNAP in a tmux session named '${SESSION}'.

  --headless    run the RobotSNAP app with -batchmode -nographics
  --robot NAME  robot the goal and the policy drive (default: ${ROBOT})
  --kill        kill an existing '${SESSION}' session, then launch a fresh one
  --stop        stop the '${SESSION}' session and launch nothing
  -h, --help    show this help

Windows: unity bridge drl_vo(2) control rviz(2) goal.
The 'rviz' window has two panes: the world view, and the markers of
tools/scene_markers.py that put the pedestrians and the goal on it.
In the 'goal' window: ${GOAL_HINT}
EOF
}

KILL=0
HEADLESS=0
STOP=0
while (($#)); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --kill)    KILL=1 ;;
        --stop)    STOP=1 ;;
        --headless) HEADLESS=1 ;;
        --robot)   ROBOT="${2:?--robot needs a name}"; shift ;;
        *) echo "run.sh: unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if ((STOP)); then
    tmux kill-session -t "${SESSION}" 2>/dev/null \
        && echo "run.sh: stopped the '${SESSION}' session"
    exit 0
fi

require_command tmux
[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ] \
    || die "run.sh: ROS ${ROS_DISTRO} is not installed; expected /opt/ros/${ROS_DISTRO}/setup.bash"
[ -x "$(venv_python)" ] \
    || die "run.sh: $(venv_python) is missing - build the workspace first"
[ -f "${MODEL}" ] \
    || die "run.sh: ${MODEL} is missing - run 'robotsnap-baseline install ${NAME}'"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    if ((KILL)); then
        tmux kill-session -t "${SESSION}"
        echo "run.sh: killed the existing '${SESSION}' session"
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

# -- drl_vo: the observation builder and the policy ----------------------------
window_new "${SESSION}" drl_vo "${HERE}"
window_run "${SESSION}" drl_vo \
    "${ENV_PREFIX} && exec $(venv_python) ${HERE}/drl_vo_observation.py --robot ${ROBOT}"
pane_split "${SESSION}:drl_vo" -h "${HERE}" \
    "${ENV_PREFIX} && exec $(venv_python) ${HERE}/drl_vo_policy.py"
tmux select-layout -t "${SESSION}:drl_vo" tiled

# -- control: hand the robot to ROS --------------------------------------------
window_control "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${ROBOT}"

# -- rviz ---------------------------------------------------------------------
# The world view and the marker pane both come from lib/common.sh now. This
# method used to start scene_markers.py itself, which is why it was the only one
# showing the crowd and the goal: the pane was a line in one run.sh rather than a
# property of the benchmark. Every method has it.
window_rviz "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${RVIZ_CONFIG}" "${ROBOT}"

# -- goal ---------------------------------------------------------------------
window_goal "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${GOAL_HINT}" "${NAME}" "${ROBOT}"

session_close "${SESSION}" "robotsnap-baseline stop ${NAME}" \
    "unity bridge drl_vo(2) control rviz(2) goal"
