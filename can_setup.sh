#!/bin/sh
echo Setting up can0 on ttyACM1
sudo slcand -o -c -s8 /dev/ttyACM1 can0
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up

