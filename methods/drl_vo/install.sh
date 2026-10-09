#!/usr/bin/env bash
#
# Prepare DRL-VO. There is nothing to build: the port is Python nodes beside
# this script, and the only artifact is the published checkpoint, which this
# script checks - or recovers from the reference image when it is missing.
#
#   install.sh                 verify the checkpoint and the interpreter
#   install.sh --from-image    recover the checkpoint from the Docker image
#   install.sh -h              this help
#
# Idempotent: a checkpoint whose hash matches the pin is left alone.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=drl_vo
MODEL="${HERE}/model/drl_vo.zip"

FROM_IMAGE=0
while [ "$#" -gt 0 ]; do
    case "$1" in
        --from-image) FROM_IMAGE=1 ;;
        -h|--help)    sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)            die "install.sh: unknown option: $1" ;;
    esac
    shift
done

verify_hash() {
    local found
    found="$(sha256sum "${MODEL}" | cut -d' ' -f1)"
    [ "${found}" = "${DRLVO_MODEL_SHA256}" ]
}

if [ -f "${MODEL}" ]; then
    if verify_hash; then
        ok "checkpoint present and pinned: ${MODEL}"
    else
        die "checkpoint at ${MODEL} does not match the pin ${DRLVO_MODEL_SHA256}; remove it and re-run (or --from-image)"
    fi
else
    [ "${FROM_IMAGE}" -eq 1 ] \
        || die "no checkpoint at ${MODEL}; re-run with --from-image to recover it from ${DRLVO_REFERENCE_IMAGE}"
    require_command docker
    info "recovering the checkpoint from ${DRLVO_REFERENCE_IMAGE}"
    mkdir -p "$(dirname "${MODEL}")"
    local_container="robotsnap_drlvo_extract_$$"
    docker create --name "${local_container}" "${DRLVO_REFERENCE_IMAGE}" >/dev/null
    # shellcheck disable=SC2064
    trap "docker rm -f '${local_container}' >/dev/null 2>&1 || true" EXIT
    docker cp "${local_container}:/root/drl_vo_ws/src/drl_vo_nav/drl_vo/src/model/drl_vo.zip" "${MODEL}"
    docker rm -f "${local_container}" >/dev/null
    trap - EXIT
    verify_hash || die "the checkpoint recovered from the image does not match the pin"
    ok "checkpoint recovered and verified: ${MODEL}"
fi

# The nodes run in the workspace virtual environment, not in the image. What the
# port needs beyond ROS is three imports: numpy for the maths, torch because the
# checkpoint is a PyTorch state dict, and stable-baselines3 because the vendored
# network subclasses one of its feature extractors.
if [ -x "$(venv_python)" ]; then
    local_missing=""
    for module in numpy torch stable_baselines3; do
        "$(venv_python)" -c "import ${module}" 2>/dev/null || local_missing="${local_missing} ${module}"
    done
    if [ -n "${local_missing}" ]; then
        warn "the virtual environment is missing:${local_missing}"
        warn "install them into $(venv_python) before running ${NAME}"
    else
        ok "interpreter carries numpy, torch and stable_baselines3"
    fi
else
    warn "no virtual environment at $(venv_python); set ROBOTSNAP_WS, or build the workspace first"
fi

info "verify the port with: robotsnap-baseline check ${NAME}"
info "start it with:         robotsnap-baseline run ${NAME}"
