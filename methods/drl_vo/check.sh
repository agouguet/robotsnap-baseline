#!/usr/bin/env bash
#
# Verify the DRL-VO port without starting a simulation.
#
# It runs selftest.py, which exercises the lidar resampling, the pedestrian map,
# the sub-goal and the command gating, then the whole chain at once, and compares
# the checkpoint's answer for known observations with the published
# implementation. Exit code is 0 when every check passes, 1 otherwise.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../../lib/common.sh
. "${HERE}/../../lib/common.sh"
source_pins

NAME=drl_vo
MODEL="${HERE}/model/drl_vo.zip"

FAILED=0
expect() {
    if [ "$1" = "0" ]; then
        ok "$2"
    else
        fail "$2"
        FAILED=1
    fi
}

[ -f "${MODEL}" ]; expect $? "checkpoint is present: ${MODEL}"
if [ -f "${MODEL}" ]; then
    [ "$(sha256sum "${MODEL}" | cut -d' ' -f1)" = "${DRLVO_MODEL_SHA256}" ]
    expect $? "checkpoint matches the pin ${DRLVO_MODEL_SHA256}"
fi
[ -d "${HERE}/vendor/gym" ]; expect $? "vendored gym shim is present"

if [ ! -x "$(venv_python)" ]; then
    fail "no interpreter at $(venv_python); set ROBOTSNAP_WS or build the workspace"
    exit 1
fi

printf '\n-- selftest --\n'
if ( cd "${HERE}" && "$(venv_python)" selftest.py ); then
    ok "selftest passes"
else
    fail "selftest failed"
    FAILED=1
fi

if [ "${FAILED}" -eq 0 ]; then
    printf '\n%s is ready: run it with "robotsnap-baseline run %s"\n' "${NAME}" "${NAME}"
else
    printf '\n%s is not ready; run "robotsnap-baseline install %s" first\n' "${NAME}" "${NAME}" >&2
fi
exit "${FAILED}"
