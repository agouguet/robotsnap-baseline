#!/usr/bin/env bash
#
# Prepare CoHAN-Nav2: a colcon workspace pinned to the verified commit, built
# on the host. Nothing is downloaded until you run this.
#
#   install.sh                 build work/cohan_nav2_ws from the pinned upstream
#   install.sh --adopt PATH    point work/cohan_nav2_ws at an existing build
#   install.sh -h              this help
#
# Idempotent: when the workspace is already built, it says so and exits. When a
# build of the same pinned commit is already on this machine, it adopts that one
# rather than spending the three minutes again.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=cohan_nav2
COHAN_WS="${COHAN_WS:-$(baseline_root)/$(method_env_value "${NAME}" WORKSPACE)}"
SETUP="${COHAN_WS}/install/setup.bash"
APT="$(method_env_value "${NAME}" APT)"

ADOPT=""
while [ "$#" -gt 0 ]; do
    case "$1" in
        --adopt)   ADOPT="${2:?--adopt needs a path}"; shift ;;
        --adopt=*) ADOPT="${1#*=}" ;;
        -h|--help) sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)         die "install.sh: unknown option: $1" ;;
    esac
    shift
done

require_command git
[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ] \
    || die "ROS ${ROS_DISTRO} is not installed; expected /opt/ros/${ROS_DISTRO}/setup.bash"

# Adopting an existing build keeps the pinned artifact and skips a rebuild. The
# workspace keeps its real location, so the absolute paths colcon wrote stay
# valid; only a symlink points at it from here.
adopt() {
    local real="$1"
    [ -f "${real}/install/setup.bash" ] \
        || die "cannot adopt ${real}: no install/setup.bash there"
    if [ -e "${COHAN_WS}" ] && [ ! -L "${COHAN_WS}" ]; then
        die "${COHAN_WS} already exists and is not a symlink; remove it first"
    fi
    mkdir -p "$(dirname "${COHAN_WS}")"
    ln -sfn "${real}" "${COHAN_WS}"
    info "adopted the existing build at ${real}"
    info "workspace: ${COHAN_WS}"
}

if [ -f "${SETUP}" ]; then
    info "already built: ${COHAN_WS}"
    exit 0
fi

if [ -n "${ADOPT}" ]; then
    adopt "$(cd "${ADOPT}" && pwd)"
    exit 0
fi

# The workspace this repository was verified against, if it is still on this
# machine: adopting it proves the pipeline on the very artifact the port was
# checked against, and saves the build.
LEGACY="$(workspace_root)/tmux/cohan_nav2_ws"
if [ -f "${LEGACY}/install/setup.bash" ]; then
    if [ "$(git -C "${LEGACY}" rev-parse HEAD 2>/dev/null)" = "${COHAN_COMMIT}" ]; then
        warn "found the verified build at ${LEGACY}; adopting it instead of rebuilding"
        warn "run 'rm ${COHAN_WS}' and re-run to force a fresh build"
        adopt "${LEGACY}"
        exit 0
    fi
    warn "${LEGACY} is not at the pinned commit ${COHAN_COMMIT}; building fresh instead"
fi

# --- a fresh build ------------------------------------------------------------
info "cloning ${COHAN_UPSTREAM} at ${COHAN_COMMIT}"
git clone "${COHAN_UPSTREAM}" "${COHAN_WS}"
git -C "${COHAN_WS}" checkout "${COHAN_COMMIT}"
git -C "${COHAN_WS}" submodule update --init --recursive

if ! dpkg -s "${APT}" >/dev/null 2>&1; then
    info "installing ${APT} (needs sudo)"
    sudo apt-get install -y "${APT}"
fi

# rosdep resolves whatever else the packages declare. It is best-effort: on a
# machine whose ROS install already carries nav2 it has nothing to add, and a
# failure here is not a reason to abandon a build that may still succeed.
if command -v rosdep >/dev/null 2>&1; then
    ( cd "${COHAN_WS}" \
        && source_ros \
        && rosdep install --from-paths src --ignore-src -r -y ) \
        || warn "rosdep did not complete; continuing, the build below will say if it matters"
fi

info "building ${COHAN_WS} (this takes a few minutes)"
(
    cd "${COHAN_WS}"
    source_ros
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
)

ok "CoHAN-Nav2 built at ${COHAN_WS}"
info "start it with: robotsnap-baseline run ${NAME}"
