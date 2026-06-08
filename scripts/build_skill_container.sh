#!/usr/bin/env bash

echo "=================================================================================================" >&2
echo "WARNING: This script is deprecated and will be removed in a future release." >&2
echo "Please transition to using the command-line tool: intrinsic_sdk_build container" >&2
echo "=================================================================================================" >&2

ARGS=()
while [[ $# -gt 0 ]]; do
  case $1 in
    --tag)
      shift
      shift
      ;;
    *)
      ARGS+=("$1")
      shift
      ;;
  esac
done

exec intrinsic_sdk_build container "${ARGS[@]}"
