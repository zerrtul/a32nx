#!/bin/bash

set -e

# get directory of this script relative to root
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

### HEY YOU!!!!
# WONDERING HOW THIS WORKS?
# Each "step" gets its own function, named "build_xyz".
# When adding a new function, make sure you invoke the
# function at the bottom with the rest.

build_instruments() {
    npm run build:instruments
}

build_behavior() {
    node "${DIR}/../src/behavior/build.js"
}

build_fbw() {
    "${DIR}/../src/fbw/build.sh"
}

build_manifests() {
    node "${DIR}/build.js"
}

build_metadata() {
    if [ -z "${GITHUB_ACTOR}" ]; then
        GITHUB_ACTOR="$(git log -1 --pretty=format:'%an <%ae>')"
    fi
    if [ -z "${GITHUB_EVENT_NAME}" ]; then
        GITHUB_EVENT_NAME="manual"
    fi
    jq -n \
        --arg built "$(date -u -Iseconds)" \
        --arg ref "$(git show-ref HEAD | awk '{print $2}')" \
        --arg sha "$(git show-ref -s HEAD)" \
        --arg actor "${GITHUB_ACTOR}" \
        --arg event_name "${GITHUB_EVENT_NAME}" \
        '{ built: $built, ref: $ref, sha: $sha, actor: $actor, event_name: $event_name }' \
        > "${DIR}/../A32NX/build_info.json"
}

if [ -z "$1" ]; then
    set -x
    build_instruments
    build_behavior
    build_fbw
else
    name="build_${1}"
    set -x
    $name
fi

# always invoke manifest+metadata because fast and useful
build_manifests
build_metadata
