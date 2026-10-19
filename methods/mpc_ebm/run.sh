#!/usr/bin/env bash
# Run MPC + EBM against a RobotSNAP build.
#
# MPC+EBM is the optimal-control end of the comparison: no learning, no
# pretrained value network - a receding-horizon solver over a crowd model its
# authors trained separately. Its authors shipped it as a ROS 2 Humble image,
# used here as it is (nothing inside it is patched), and the three things it
# cannot know are adapted beside it:
#
#   tools/odom_relay.py     /odom -> /robot_odom           (same message)
#   tools/agents_adapter.py /simulation/agents -> /social_sim/agents, world frame
#   this file               /social_sim/scene_info, the map the nodes plan in
#
# One tmux session, one window per responsibility:
#   unity    - the RobotSNAP player alone, without the bridge
#   bridge   - "python -m robotsnap bridge --ros2"
#   method   - the MPC+EBM container and the three adapters that feed it
#   control  - hands the robot to ROS, so /cmd_vel is what drives it
#   rviz     - the shared view, tools/scene.rviz
#   goal     - the scene the method plans in, and how to change it
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=mpc_ebm
SESSION=mpc_ebm
ROBOT=robot_1

IMAGE="$(method_env_value "${NAME}" IMAGE)"
CONTAINER="$(method_env_value "${NAME}" CONTAINER)"
SCENES="$(method_env_value "${NAME}" SCENES)"
SCENE="$(method_env_value "${NAME}" DEFAULT_SCENE)"
GOAL_HINT="$(method_env_value "${NAME}" GOAL_HINT)"
RVIZ_CONFIG="$(baseline_root)/tools/scene.rviz"

# The image's entry point for an external simulator. Despite the name of the
# launch it starts no Gazebo: this is the "sbrn" demo, the variant that reads
# /robot_odom, the crowd and the scene from outside and publishes /cmd_vel back.
# Kept as one string so `run.sh --print-method` and the tmux pane can only ever
# show the same command.
container_cmd() {
    printf 'docker run --rm --name %s --network host -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 --entrypoint "" %s bash -c %s' \
        "${CONTAINER}" "${IMAGE}" \
        "'sleep 1 && source /opt/ros/humble/setup.bash && source /root/mpc-ebm_ws/install/setup.bash && cd /root/mpc-ebm_ws/ && stdbuf -o L ros2 launch sbrn_demo demo_warehouse_single_robot_gazebo_mpc_mmp.launch.py'"
}

# The image's map nodes wait for two messages RobotSNAP does not publish:
# `scene_info` names the map to load, and `new_scenario` asks for it. The scene
# name has to be one of the benchmark maps because the method plans in that
# map, in metres, not in RobotSNAP's frame - the two are only the same map when
# the scenario being played is the benchmark one. `scene_info` is repeated so a
# node that starts after Unity still learns the scene; it is idempotent, the
# map nodes reload only when the name changes.
scene_cmd() {
    printf '( ros2 topic pub -r 1 /social_sim/scene_info simulation_msgs/msg/SceneInfo "{environment: %s}" & sleep 3; for _ in 1 2 3; do ros2 topic pub --once /new_scenario std_msgs/msg/String "{data: %s}"; sleep 1; done; wait )' \
        "'${SCENE}'" "'${SCENE}'"
}

usage() {
    cat <<EOF
Usage: run.sh [--headless] [--kill] [--stop] [--robot NAME] [--scene NAME]
              [--print-method] [--help]

Start MPC+EBM and RobotSNAP in a tmux session named '${SESSION}'.

  --headless        run the RobotSNAP app with -batchmode -nographics
  --scene NAME      benchmark map the method plans in (default: ${SCENE})
                    one of: ${SCENES}
  --robot NAME      robot /cmd_vel drives (default: ${ROBOT}; the image
                    hardcodes the bare topic names, so only robot_1 is supported)
  --kill            kill an existing '${SESSION}' session and its container
  --stop            stop the '${SESSION}' session and its container
  --print-method    print the container command and exit
  -h, --help        show this help

Windows: unity bridge method(4) control rviz(2) goal.
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
        --scene)        SCENE="${2:?--scene needs a name}"; shift ;;
        --robot)        ROBOT="${2:?--robot needs a name}"; shift ;;
        *) echo "run.sh: unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

case " ${SCENES} " in
    *" ${SCENE} "*) ;;
    *) die "run.sh: unknown scene '${SCENE}'; the image carries: ${SCENES}" ;;
esac

if ((PRINT_METHOD)); then
    container_cmd; echo; exit 0
fi

if [ "${ROBOT}" != "robot_1" ]; then
    die "run.sh: only robot_1 is supported: the image grounds its odometry on /robot_odom and its velocity on /cmd_vel, and this method does not ship the topic alias another robot would need"
fi

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

# -- method: the container, and the adapters that make it see RobotSNAP --------
window_new "${SESSION}" method "${HERE}"
window_run "${SESSION}" method "$(container_cmd)"
pane_split "${SESSION}:method" -h "${HERE}" \
    "${ENV_PREFIX} && exec $(venv_python) $(baseline_root)/tools/odom_relay.py"
pane_split "${SESSION}:method" -v "${HERE}" \
    "${ENV_PREFIX} && exec $(venv_python) $(baseline_root)/tools/agents_adapter.py"
pane_split "${SESSION}:method" -v "${HERE}" \
    "${ENV_PREFIX} && exec bash -lc $(printf '%q' "$(scene_cmd)")"
tmux select-layout -t "${SESSION}:method" tiled

# -- control: hand the robot to ROS --------------------------------------------
window_control "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${ROBOT}"

# -- rviz ---------------------------------------------------------------------
window_rviz "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${RVIZ_CONFIG}" "${ROBOT}"

# -- goal ---------------------------------------------------------------------
window_goal "${SESSION}" "${HERE}" "${ENV_PREFIX}" "${GOAL_HINT}" "${NAME}" "${ROBOT}"

session_close "${SESSION}" "robotsnap-baseline stop ${NAME}" \
    "unity bridge method(4) control rviz(2) goal"
