#!/usr/bin/env bash
# Sets latency of FT232 usb-serial
# Argument: Device file or symlink, e.g. /dev/robot/ttyIgusD
# This could be run with udev or udev could set ATTR{latency_timer}="1"

function die {
echo $1 ; exit 1
}

# Get real name of device
[ -L "$1" ] && DEV=$(readlink -f "$1") || DEV="$1"
[ -c "$DEV" ] || die "Device $DEV is not a character device!"
DEV=$(basename $DEV)

FILE_LATENCY=/sys/bus/usb-serial/devices/$DEV/latency_timer
[ -f $FILE_LATENCY ] || die "Sysfs latebcy file $FILE_LATENCY not found!"

echo 1 | sudo tee $FILE_LATENCY
echo -n "Device $DEV latency: "
cat $FILE_LATENCY