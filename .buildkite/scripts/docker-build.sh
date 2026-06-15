#!/usr/bin/env bash

set -euo pipefail

target="${1:-runtime}"
commit="${BUILDKITE_COMMIT:-local}"
tag="${commit:0:12}"

case "$target" in
  runtime)
    image="openmowernext"
    dockerfile="Dockerfile"
    context="."
    ;;
  devcontainer)
    image="openmowernext-devcontainer"
    dockerfile=".devcontainer/Dockerfile"
    context=".devcontainer"
    ;;
  *)
    echo "unknown docker build target: $target" >&2
    exit 2
    ;;
esac

docker build \
  --pull \
  --file "$dockerfile" \
  --tag "${image}:buildkite-${tag}" \
  "$context"
