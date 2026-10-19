#!/usr/bin/env bash
#
# Prepare nav2 + DWB. Nothing is downloaded and nothing is built: the stack and
# the plugins are packages of the ROS 2 installation this workspace already
# needs. This script only says whether they are here.
#
#   install.sh       check the nav2 packages
#   install.sh -h    this help

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help) sed -n '2,8p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)         die "install.sh: unknown option: $1" ;;
    esac
done

missing=""
for package in nav2_bringup dwb_core dwb_critics nav2_navfn_planner; do
    if [ -d "/opt/ros/${ROS_DISTRO}/share/${package}" ]; then
        ok "nav2 package present: ${package}"
    else
        fail "nav2 package missing: ${package}"
        missing="${missing} ${package}"
    fi
done

if [ -n "${missing}" ]; then
    die "install the missing packages, then re-run: sudo apt install ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-dwb-core ros-${ROS_DISTRO}-dwb-critics ros-${ROS_DISTRO}-nav2-navfn-planner"
fi

info "verify the method with: robotsnap-baseline check nav2_dwb"
info "start it with:         robotsnap-baseline run nav2_dwb"
