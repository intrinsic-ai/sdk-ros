#!/bin/sh

for patch_file in "$@"; do
  echo "Applying patch file $(basename ${patch_file})"
  patch_out="$(patch -b -p1 < ${patch_file})"
  result=$?

  if [ $result -ne 0 ]; then
    echo "${patch_out}" | grep "Reversed (or previously applied) patch detected!" -q
    if [ $? -eq 0 ]; then
      result=0
    else
      echo "${patch_out}"
      exit $result
    fi
  fi
done

exit $result
