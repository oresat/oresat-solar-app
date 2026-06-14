#!/bin/bash
n=${1:-0}
echo opening can0 on /dev/ttyACM$n
sudo slcand -o -c -s8 /dev/ttyACM$n can0
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 up

