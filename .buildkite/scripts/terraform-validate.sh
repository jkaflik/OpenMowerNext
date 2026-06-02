#!/usr/bin/env bash

set -euo pipefail

docker run --rm \
  --volume "$PWD:/workspace" \
  --workdir /workspace \
  --entrypoint sh \
  hashicorp/terraform:1.9.8 \
  -lc '
    set -eu
    terraform -chdir=infra/buildkite fmt -check
    terraform -chdir=infra/buildkite init -backend=false -input=false
    terraform -chdir=infra/buildkite validate
  '
