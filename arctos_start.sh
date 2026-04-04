#!/bin/bash

# 1. Set permissions for the USB device
echo "Setting permissions for /dev/ttyACM0..."
sudo chmod 666 /dev/ttyACM0

# 2. Kill any existing slcan processes to prevent conflicts
sudo pkill slcand
sleep 1

# 3. Attach the CANable adapter as a network interface (slcan)
# -s6 sets the bitrate to 500k to match your rosjog-2.py config
echo "Initializing SLCAN at 500k bitrate..."
sudo slcand -o -c -f -s6 /dev/ttyACM0 can0
sleep 1

# 4. Bring the can0 interface UP
echo "Bringing can0 interface online..."
sudo ip link set can0 up

echo "--- Hardware Ready ---"

# 5. Launch the ROS Driver (rosjog-2.py)
# Make sure you are in the correct directory or provide the full path
#need to change this based on what actual file is called
python3 rosjogactual.py
