#!/usr/bin/env bash
# Apply patches/external/<repo>/*.patch onto src/external/<repo>. Idempotent:
# a patch that is already applied (git apply --check --reverse succeeds) is skipped.
# Patches are made with `git format-patch` inside the external checkout and are
# the ONLY allowed way to change external sources (keeps runs reproducible).
set -euo pipefail
ws=$(cd "$(dirname "$0")/.." && pwd)
shopt -s nullglob
for dir in "$ws"/patches/external/*/; do
  repo=$(basename "$dir")
  target="$ws/src/external/$repo"
  [ -d "$target" ] || { echo "skip $repo: not imported"; continue; }
  for p in "$dir"*.patch; do
    if git -C "$target" apply --check --reverse "$p" >/dev/null 2>&1; then
      echo "already applied: $repo/$(basename "$p")"
    else
      git -C "$target" apply --check "$p"
      git -C "$target" apply "$p"
      echo "applied: $repo/$(basename "$p")"
    fi
  done
done
