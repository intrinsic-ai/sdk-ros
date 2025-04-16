#!/bin/sh

echo "Applying patch file $(basename $1)"
patch_out="$(patch -b -p1 < $1)"
result=$?

if [ $result -ne 0 ]; then
	echo "${patch_out}" | grep "Reversed (or previously applied) patch detected!" -q
	if [ $? -eq 0 ]; then
		result=0
	else
		echo "${patch_out}"
	fi
fi

return $result
