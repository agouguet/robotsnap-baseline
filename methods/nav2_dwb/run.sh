#!/usr/bin/env bash
# Run nav2 with the DWB local planner against a RobotSNAP build.
#
# This method is the classical end of the comparison: no container, no model,
# no notion of people. Three shared tools make nav2 fit RobotSNAP:
#
#   tools/play_scenario.py starts the scenario, so the launch has a robot to
#                          drive instead of a player in standby
#   tools/wait_for_robot.py holds nav2 back until that robot's frames resolve -
#                          a stack started earlier stays inactive in silence
#   (no clock relay)       the gateway publishes /clock itself, on simulated
#                          time; a second publisher on the same topic makes the
#                          clock jump back and every node clear its TF buffer
#   tools/frames_alias.py  the frame names nav2 hardcodes (base_link), as
#                          identity links from RobotSNAP's chassis_link
#
#   unity    - the RobotSNAP player alone, without the bridge
#   bridge   - "python -m robotsnap bridge --ros2"
#   sim      - starts the scenario (skipped with --no-play)
#   nav2     - the wait gate, then the stack, and the frame aliases beside it
#   control  - hands the robot to ROS, so /cmd_vel is what drives it
#   rviz     - the shared view, tools/scene.rviz
#   goal     - a shell ready to run tools/send_goal_nav2.py
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=nav2_dwb
SESSION=nav2_dwb
ROBOT=robot_1

PARAMS="${HERE}/nav2_dwb_params.yaml"
RVIZ_CONFIG="$(baseline_root)/tools/scene.rviz"
GOAL_HINT="$(method_env_value "${NAME}" GOAL_HINT)"
LAUNCH_CMD="ros2 launch nav2_bringup navigation_launch.py params_file:=${PARAMS} use_sim_time:=true autostart:=true"

usage() {
    cat <<EOF
Usage: run.sh [--headless] [--kill] [--stop] [--no-play] [--help]

Start nav2 (DWB) and RobotSNAP in a tmux session named '${SESSION}'.

  --headless   run the RobotSNAP app with -batchmode -nographics
  --kill       kill an existing '${SESSION}' session, then launch a fresh one
  --stop       stop the '${SESSION}' session and launch nothing
  --no-play    do not start the scenario; wait for one started by hand
  -h, --help   show this help

The 'sim' window asks the player to reset and play, so the launch begins on a
fresh episode. The 'nav2' window waits for the robot's frames before starting
the stack: a stack that starts first sits inactive without saying why.

Windows: unity bridge nav2(2) control rviz(2) goal(2).
In the 'goal' window: ${GOAL_HINT}
EOF
}

KILL=0
HEADLESS=0
STOP=0
PLAY=1
while (($#)); do
    case "$1" in
        -h|--help)  usage; exit 0 ;;
        --kill)     KILL=1 ;;
        --stop)     STOP=1 ;;
        --headless) HEADLESS=1 ;;
        --no-play)  PLAY=0 ;;
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
[ -f "${PARAMS}" ] || die "run.sh: ${PARAMS} is missing"

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

# -- sim: start the scenario, so there is a robot to drive ---------------------
if ((PLAY)); then
    window_new "${SESSION}" sim "${HERE}"
    window_run "${SESSION}" sim \
        "${ENV_PREFIX} && $(venv_python) $(baseline_root)/tools/play_scenario.py && echo 'scenario running' || echo 'could not start the scenario; start it in the application'"
fi

# -- nav2: the stack, once the robot's frames resolve, plus the frame aliases ---
window_new "${SESSION}" nav2 "${HERE}"
window_run "${SESSION}" nav2 \
    "${ENV_PREFIX} && $(venv_python) $(baseline_root)/tools/wait_for_robot.py && exec ${LAUNCH_CMD}"
pane_split "${SESSION}:nav2" -h "${HERE}" \
    "${ENV_PREFIX} && exec $(venv_python) $(baseline_root)/tools/frames_alias.py"
tmux select-layout -t "${SESSION}:nav2" tiled

# -- control: hand the robot to ROS --------------------------------------------
window_control "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${ROBOT}"

# -- rviz ---------------------------------------------------------------------
window_rviz "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${RVIZ_CONFIG}" "${ROBOT}"

# -- goal ---------------------------------------------------------------------
window_goal "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${GOAL_HINT}" "${NAME}" "${ROBOT}"

if ((PLAY)); then
    WINDOWS="unity bridge sim nav2(2) control rviz(2) goal(2)"
else
    WINDOWS="unity bridge nav2(2) control rviz(2) goal(2)"
fi
session_close "${SESSION}" "robotsnap-baseline stop ${NAME}" "${WINDOWS}"
