#!/bin/sh
echo Tearing down can0
sudo ifconfig can0 down
sudo ip link delete can0 type can
sudo killall slcand

