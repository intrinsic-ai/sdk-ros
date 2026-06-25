#!/bin/sh
SCRIPT_DIR=$(dirname "$0")
${SCRIPT_DIR}/apply_patch.sh \
  ${SCRIPT_DIR}/001_zenoh_helpers_cc_no_runfiles.patch \
  ${SCRIPT_DIR}/002_gcc13_publisher_noexcept.patch \
  ${SCRIPT_DIR}/003_fix_return_types.patch \
  ${SCRIPT_DIR}/003_icon_strerror_glibc2_39.patch \
  ${SCRIPT_DIR}/004_add_libimw_zenoh_so_target.patch

python3 ${SCRIPT_DIR}/fix_cwiseLess.py
