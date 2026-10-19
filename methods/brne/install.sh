#!/usr/bin/env bash
#
# Prepare BRNE. There is nothing to build: the method is a Docker image its
# authors built, and this script only makes sure that image is here.
#
#   install.sh           check the image, pulling it when it is missing
#   install.sh -h        this help
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=brne
IMAGE="$(method_env_value "${NAME}" IMAGE)"

while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help) sed -n '2,7p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)         die "install.sh: unknown option: $1" ;;
    esac
done

require_command docker

if docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    ok "image present: ${IMAGE}"
else
    info "image absent, pulling ${IMAGE} (several GB)"
    docker pull "${IMAGE}"
    ok "image pulled: ${IMAGE}"
fi

# The adapters speak agents_msgs, which the legacy ROS2 workspace builds. Its
# absence is the one thing that would make a launch fail with an ImportError
# rather than a message, so it is checked here.
if [ -x "$(venv_python)" ]; then
    if ( source_ros >/dev/null 2>&1; source_setup "$(workspace_root)/$(method_env_value "${NAME}" SOURCES)" >/dev/null 2>&1; \
         "$(venv_python)" -c "import agents_msgs" 2>/dev/null ); then
        ok "interpreter carries agents_msgs"
    else
        warn "agents_msgs is not importable; build the ROS2 workspace that carries it"
    fi
else
    warn "no virtual environment at $(venv_python); set ROBOTSNAP_WS, or build the workspace first"
fi

info "verify the method with: robotsnap-baseline check ${NAME}"
info "start it with:         robotsnap-baseline run ${NAME}"
