#!/usr/bin/env bash
# Launch CoHAN-Nav2 (HATEB under nav2) against a RobotSNAP build.
#
# CoHAN-Nav2 is a ROS 2 / nav2 stack and a plain colcon workspace on the host:
# no container and no ros1_bridge. One tmux session, one window per
# responsibility:
#   unity    - the RobotSNAP player alone, without the bridge
#   bridge   - "python -m robotsnap bridge --ros2"
#   cohan    - the nav2 stack (planner, controller, bt_navigator, behaviors)
#   agents   - our /tracked_agents bridge, the clock and the frame aliases
#   cohan_agents - CoHAN-Nav2's own crowd nodes
#   control  - hands the robot to ROS, so /cmd_vel is what drives it
#   rviz     - rviz2 with CoHAN's own configuration
#   goal     - a shell ready to run send_goal.py
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=cohan_nav2
SESSION=cohan
# The whole stack is wired to the primary robot's bare topic names - the
# parameters, the agents bridge and the frame aliases all read `/odom` and
# `/scan` - so the robot handed to ROS is that one, and there is no flag to
# point it elsewhere while those stay fixed.
ROBOT=robot_1

COHAN_WS="${COHAN_WS:-$(baseline_root)/$(method_env_value "${NAME}" WORKSPACE)}"
COHAN_SETUP="${COHAN_WS}/install/setup.bash"
PARAMS="${HERE}/cohan_robotsnap_params.yaml"
RVIZ_CONFIG="${COHAN_WS}/install/cohan_sim_navigation/share/cohan_sim_navigation/rviz/cohan.rviz"
GOALS_FILE="${COHAN_WS}/install/agent_path_prediction/share/agent_path_prediction/cfg/goals_adream.yaml"
NAV_CMD="ros2 launch cohan_sim_navigation navigation_launch.py params_file:=${PARAMS} use_sim_time:=true autostart:=true use_composition:=false"
GOAL_HINT="$(method_env_value "${NAME}" GOAL_HINT)"

usage() {
    cat <<EOF
Usage: run.sh [--headless] [--kill] [--stop] [--help]

Start CoHAN-Nav2 and RobotSNAP in a tmux session named '${SESSION}'.

  --headless   run the RobotSNAP app with -batchmode -nographics
  --kill       kill an existing '${SESSION}' session, then launch a fresh one
  --stop       stop the '${SESSION}' session and launch nothing
  -h, --help   show this help

Windows: unity bridge cohan agents cohan_agents control rviz goal.
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
if [ ! -f "${COHAN_SETUP}" ]; then
    die "run.sh: ${COHAN_SETUP} is missing - build the workspace first:
  robotsnap-baseline install ${NAME}"
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    if ((KILL)); then
        tmux kill-session -t "${SESSION}"
        echo "run.sh: killed the existing '${SESSION}' session"
    else
        die "run.sh: tmux session '${SESSION}' already exists; re-run with --kill to replace it"
    fi
fi

# ROS, then the setups the method declares, then CoHAN's own workspace: later
# sources win, so the stack sits on top of the overlay chain.
mapfile -t METHOD_SOURCES < <(method_sources "${NAME}")
ENV_PREFIX="$(ros_env "${HERE}" "${METHOD_SOURCES[@]}" "${COHAN_SETUP}")"

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

# -- cohan: the navigation stack ----------------------------------------------
window_new "${SESSION}" cohan "${HERE}"
window_run "${SESSION}" cohan "${ENV_PREFIX} && exec ${NAV_CMD}"

# -- agents: what the stack needs from RobotSNAP ------------------------------
window_new "${SESSION}" agents "${HERE}"
window_run "${SESSION}" agents "${ENV_PREFIX} && exec $(venv_python) ${HERE}/agents_bridge.py"
pane_split "${SESSION}:agents" -h "${HERE}" \
    "${ENV_PREFIX} && exec $(venv_python) ${HERE}/clock_relay.py"
pane_split "${SESSION}:agents" -v "${HERE}" \
    "${ENV_PREFIX} && exec $(venv_python) ${HERE}/frames_alias.py"
tmux select-layout -t "${SESSION}:agents" tiled

# -- cohan_agents: CoHAN-Nav2's own crowd nodes -------------------------------
window_new "${SESSION}" cohan_agents "${HERE}"
window_run "${SESSION}" cohan_agents \
    "${ENV_PREFIX} && exec ros2 run agent_path_prediction agent_path_predict --ros-args -p use_sim_time:=true -p robot_frame_id:=chassis_link -p get_plan_srv_name:=/compute_path_to_pose -p goals_file:=${GOALS_FILE}"
# agent_filter.py reads sys.argv[1] as a namespace. Passed no argument at all it
# takes the first leftover token - the '--ros-args' itself - and builds the topic
# '/--ros-args/scan'. An explicit empty namespace is the one spelling that works
# whether or not rclpy has already stripped the ROS arguments.
pane_split "${SESSION}:cohan_agents" -h "${HERE}" \
    "${ENV_PREFIX} && exec ros2 run invisible_humans_detection invisible_humans_detection_node --ros-args -p use_sim_time:=true"
pane_split "${SESSION}:cohan_agents" -v "${HERE}" \
    "${ENV_PREFIX} && exec ros2 run cohan_layers agent_filter.py '' --ros-args -p use_sim_time:=true"
tmux select-layout -t "${SESSION}:cohan_agents" tiled

# -- control: hand the robot to ROS --------------------------------------------
# nav2 plans and publishes /cmd_vel, but the robot only reads it in its ROS
# mode. Without this the stack looks healthy and the robot never moves, which
# is the same trap DRL-VO's control window closes.
window_control "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${ROBOT}"

# -- rviz ---------------------------------------------------------------------
window_rviz "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${RVIZ_CONFIG}"

# -- goal ---------------------------------------------------------------------
window_goal "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${GOAL_HINT}"

session_close "${SESSION}" "robotsnap-baseline stop ${NAME}" \
    "unity bridge cohan agents cohan_agents control rviz goal"
