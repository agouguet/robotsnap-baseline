#!/usr/bin/env bash
#
# Verify nav2 + DWB without starting a simulation.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

FAILED=0
expect() {
    if [ "$1" = "0" ]; then
        ok "$2"
    else
        fail "$2"
        FAILED=1
    fi
}

[ -f "/opt/ros/${ROS_DISTRO}/share/nav2_bringup/launch/navigation_launch.py" ]
expect $? "nav2_bringup ships navigation_launch.py"
[ -f "/opt/ros/${ROS_DISTRO}/lib/libdwb_core.so" ]
expect $? "the DWB controller plugin is installed"
[ -f "${HERE}/nav2_dwb_params.yaml" ]
expect $? "the parameters file is present"

if [ -x "$(venv_python)" ]; then
    ( "$(venv_python)" -c "
import sys, yaml
document = yaml.safe_load(open(sys.argv[1]))
assert 'controller_server' in document, 'no controller_server section'
assert document['controller_server']['ros__parameters']['controller_plugins'] == ['FollowPath']
plugins = document['controller_server']['ros__parameters']['FollowPath']['plugin']
assert plugins == 'dwb_core::DWBLocalPlanner', plugins
" "${HERE}/nav2_dwb_params.yaml" ) 2>/dev/null
    expect $? "the parameters select the DWB planner"
else
    warn "no interpreter at $(venv_python); skipping the parameters check"
fi

# clock_relay.py is deliberately absent from this list: the gateway publishes
# /clock itself, and adding a second writer is what made the stack read a clock
# that jumped backwards.
for tool in frames_alias.py send_goal_nav2.py scenario_goal.py scene_markers.py; do
    [ -f "$(baseline_root)/tools/${tool}" ]
    expect $? "shared tool is present: ${tool}"
done

if [ "${FAILED}" -eq 0 ]; then
    printf '\nnav2_dwb is ready: run it with "robotsnap-baseline run nav2_dwb"\n'
else
    printf '\nnav2_dwb is not ready; run "robotsnap-baseline install nav2_dwb" first\n' >&2
fi
exit "${FAILED}"
