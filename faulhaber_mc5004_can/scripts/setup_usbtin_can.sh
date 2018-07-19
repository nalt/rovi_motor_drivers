#!/bin/bash

# This script initializes a USBtin device (http://www.fischl.de/usbtin/) and prepares it for use with KaCanOpen's socket driver.

command_exists(){
	[ -x "$(command -v $1)" ]
}

check_for_command() {
	if ! command_exists $1; then
		echo "ERROR: Please install $1." >&2
		exit 1
	fi
}

check_for_command "git"

# roslaunch adds two additional arguments
if [[ $# -lt 3 ]] ; then
  echo "Please specify tty device name and optionally baudrate as arguments"
fi


if [[ $# -eq 3 ]] ; then
  tty_device=$1
	baudrate=6
fi

if [[ $# -eq 4 ]] ; then
	tty_device=$1

	if [[$4 -eq 10000]]  ; then baudrate=0
	elif [[$4 -eq 20000]] ; then baudrate=1
	elif [[$4 -eq 50000]] ; then baudrate=2
	elif [[$4 -eq 100000]] ; then baudrate=3
	elif [[$4 -eq 125000]] ; then baudrate=4
	elif [[$4 -eq 250000]] ; then baudrate=5
	elif [[$4 -eq 500000]] ; then baudrate=6
	elif [[$4 -eq 800000]] ; then baudrate=7
  elif [[$4 -eq 1000000]] ; then baudrate=8
	else baudrate=6
	fi
fi

echo "Setting up device $tty_device with baudrate $baudrate"


SCRIPTDIR=$(dirname "$(readlink -f "$0")")

cd "$SCRIPTDIR"

if [ ! -d "$SCRIPTDIR/can-utils" ]; then
	echo "can-utils not yet available."
	echo "Cloning and building now."
	git clone https://github.com/linux-can/can-utils
	cd "$SCRIPTDIR/can-utils"
	make
fi

cd "$SCRIPTDIR/can-utils"

# Setting up the device
sudo killall slcand
sudo ./slcan_attach -f -s$baudrate -nslcan0 -o /dev/$tty_device
sudo ./slcand $tty_device slcan0
sudo ifconfig slcan0 txqueuelen 1000
sudo ip link set slcan0 up
