#!/bin/bash


sudo modprobe vcan

sudo ip link add dev vcan0 type vcan

sudo ip link set up vcan0

echo "vcan0 interface is set up and ready to use."
