#!/usr/bin/env python3
"""
zero_motors.py — Set current position as zero for all Arctos motors.
Run this with the arm physically at your desired home position.
"""

import can

CAN_INTERFACE = "slcan"
CAN_CHANNEL   = "/dev/ttyACM0"
CAN_BITRATE   = 500000
CAN_IDS       = [7, 8, 9, 10, 11, 12]

def set_zero(bus, can_id):
    data = [0x93, 0x88]
    crc  = (can_id + sum(data)) & 0xFF
    data.append(crc)
    msg = can.Message(
        arbitration_id=can_id,
        data=data,
        is_extended_id=False
    )
    bus.send(msg)
    print(f"Zero set for motor CAN ID {can_id}: data={[hex(b) for b in data]}")

def main():
    bus = can.interface.Bus(
        interface=CAN_INTERFACE,
        channel=CAN_CHANNEL,
        bitrate=CAN_BITRATE
    )
    print("Setting zero position for all motors...")
    for can_id in CAN_IDS:
        set_zero(bus, can_id)
    print("Done. All motors zeroed at current position.")
    bus.shutdown()

if __name__ == "__main__":
    main()
