#!/bin/bash
#
# This script is ran as a part of the /etc/rc.local run configuration.
# Therefore, it is ran with root permissions already.
# (i.e., this happens on every boot)
#

/home/nvidia/jetson_clocks.sh
iw dev wlan0 set power_save off
