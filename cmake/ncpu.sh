#! /bin/sh

UNAME=$(uname)
if [ $UNAME = 'Darwin' ]; then
	echo $(sysctl -n hw.logicalcpu_max)
elif [ $UNAME = 'Linux' ]; then
	echo $(lscpu -p | egrep -v '^#' | sort -u -t, -k 2,4 | wc -l)
fi
