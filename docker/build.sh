#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
IMAGE=rayneo-sdk-xwin:local

echo "Building RayneoSDK v${SDK_VERSION}..."

docker build -t "$IMAGE" "$ROOT/docker"
docker run --rm \
    --user "$(id -u):$(id -g)" \
    -e "SDK_VERSION=${SDK_VERSION}" \
    -v "$ROOT:/work" \
    -w /work \
    "$IMAGE" \
    ./docker/build-in-container.sh
