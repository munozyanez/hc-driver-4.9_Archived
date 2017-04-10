#!/bin/sh

# Get major number from /proc/devices
MAJOR=`awk '/hcanpci/{print $1}' /proc/devices`
test -z $MAJOR && exit 1

# Get the number of CAN nodes (could be a lot if you have multiple boards on
# the system)
NUM_NODES=`ls /proc/hcanpci/can* | wc -w`
if [ $NUM_NODES -eq 0 ]; then
    echo "$0: no can devices found" >&2
    exit 1
fi

# remove old nodes
rm /dev/can[0-9]* 2>/dev/null

for NODE in /proc/hcanpci/can[0-9]*; do
	NODE=$(basename $NODE)
	MINOR=${NODE##*[a-z]}
	
	test ! -e /dev/can$MINOR && \
		mknod /dev/can$MINOR c $MAJOR $MINOR && \
		chmod 0666 /dev/can$MINOR
done

exit 0
