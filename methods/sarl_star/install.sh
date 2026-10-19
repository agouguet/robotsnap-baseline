#!/usr/bin/env bash
#
# Prepare SARL*. There is nothing to build: the method is the Docker image its
# authors built, and the ROS 1 side is tools/ros1/compose.yaml. This script only
# makes sure the images a launch needs are here.
#
#   install.sh           check the images, pulling the ones that can be pulled
#   install.sh -h        this help
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=sarl_star
IMAGE="$(method_env_value "${NAME}" IMAGE)"

while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help) sed -n '2,9p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)         die "install.sh: unknown option: $1" ;;
    esac
done

require_command docker

# An image that is here is left alone; one that is not is pulled when it can be,
# and named as missing when it cannot. The ROS 1 bridge image is built locally,
# so it is the one that has to be pointed at rather than fetched.
ensure_image() {
    local image="$1" pullable="$2"
    if docker image inspect "${image}" >/dev/null 2>&1; then
        ok "image present: ${image}"
        return 0
    fi
    if [ "${pullable}" = "1" ]; then
        info "image absent, pulling ${image} (several GB)"
        docker pull "${image}"
        ok "image pulled: ${image}"
    else
        fail "image absent and not pullable: ${image}"
        fail "build it, or load it from a tarball, then re-run"
        return 1
    fi
}

ensure_image "${IMAGE}" 1
ensure_image "${ROS1_MASTER_IMAGE:-ros:noetic}" 1
ensure_image "${ROS1_BRIDGE_IMAGE:?pins.env must define ROS1_BRIDGE_IMAGE}" 0

if [ -x "$(venv_python)" ]; then
    ok "workspace interpreter present: $(venv_python)"
else
    warn "no virtual environment at $(venv_python); set ROBOTSNAP_WS, or build the workspace first"
fi

info "verify the method with: robotsnap-baseline check ${NAME}"
info "start it with:         robotsnap-baseline run ${NAME}"
