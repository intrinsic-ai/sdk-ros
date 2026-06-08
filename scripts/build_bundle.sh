#!/usr/bin/env bash

echo "=================================================================================================" >&2
echo "WARNING: This script is deprecated and will be removed in a future release." >&2
echo "Please transition to using the command-line tool: intrinsic_sdk_build bundle" >&2
echo "=================================================================================================" >&2

exec intrinsic_sdk_build bundle "$@"
