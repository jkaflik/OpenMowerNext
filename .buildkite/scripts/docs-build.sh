#!/usr/bin/env bash

set -euo pipefail

docker run --rm \
  --volume "$PWD:/workspace" \
  --workdir /workspace/docs \
  node:18-bookworm \
  bash -lc '
    set -euo pipefail
    npm ci
    npm run docs:build
    touch .vitepress/dist/.nojekyll
  '
