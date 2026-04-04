#!/bin/bash
echo "Bringing up CAN interface..."
sudo killall slcand 2>/dev/null
sleep 0.5
sudo slcand -o -s6 -t hw -S 3000000 /dev/canable can0
sleep 0.5
sudo ip link set can0 up
ip link show can0 | grep -q "UP" && echo "✓ can0 is UP" || echo "✗ can0 failed"
