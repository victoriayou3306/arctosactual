#!/usr/bin/env python3
"""
joint_state_bridge.py
=====================
Arctos GUI → RViz visualizer bridge.

Listens to live CAN traffic on can0 (slcan via CANable),
decodes motor position commands, converts to joint angles
using your gear ratios, and publishes to /joint_states
so RViz/MoveIt animates the arm in real time.

Usage (in a separate terminal, after roslaunch arctos_config demo.launch):
    python3 joint_state_bridge.py

Requirements:
    pip3 install python-can[serial]
    ROS Noetic with sensor_msgs

Author: generated for Arctos CANable + ROS Noetic setup
"""

import can
import rospy
import math
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# ─── CONFIG ──────────────────────────────────────────────────────────────────

# CANable slcan connection
CAN_BUSTYPE = "socketcan"
CAN_CHANNEL  = "can0"
CAN_BITRATE  = 1000000

# Gear ratios per joint (update if your arm differs)
# Joint order: J1, J2, J3, J4, J5, J6
GEAR_RATIOS = [6.75, 75, 75, 24, 33.91, 33.91]

# Steps per revolution of each motor (16 microsteps typical)
STEPS_PER_REV = 3200  # 200 steps * 16 microsteps

# CAN arbitration IDs for each joint (0x01 = joint1 ... 0x06 = joint6)
# Adjust if your motor IDs differ
JOINT_CAN_IDS = {
    0x01: 0,  # Joint 1
    0x02: 1,  # Joint 2
    0x03: 2,  # Joint 3
    0x04: 3,  # Joint 4
    0x05: 4,  # Joint 5
    0x06: 5,  # Joint 6
}

# ROS joint names — must match your URDF exactly
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "jaw1", "jaw2"]

# How often to publish joint states (seconds)
PUBLISH_RATE = 0.05  # 20Hz

# ─── STATE ───────────────────────────────────────────────────────────────────

# Accumulated step counts per joint (starts at 0 = home)
joint_steps   = [0] * 8
joint_angles  = [0.0] * 8  # radians

# ─── HELPERS ─────────────────────────────────────────────────────────────────

def steps_to_radians(steps: int, joint_idx: int) -> float:
    """Convert accumulated motor steps to joint angle in radians."""
    gear = GEAR_RATIOS[joint_idx]
    revs = steps / (STEPS_PER_REV * gear)
    return revs * 2.0 * math.pi


def decode_steps_from_can(msg: can.Message) -> int:
    """
    Extract step count from a CAN message.
    Arctos protocol: bytes [1..4] encode a signed 32-bit step value.
    Adjust this if your firmware uses a different layout.
    """
    if len(msg.data) < 5:
        return None
    # Signed 32-bit big-endian from bytes 1-4
    raw = (msg.data[1] << 24) | (msg.data[2] << 16) | (msg.data[3] << 8) | msg.data[4]
    # Convert to signed
    if raw > 0x7FFFFFFF:
        raw -= 0x100000000
    return raw


# ─── MAIN ────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node("arctos_joint_state_bridge", anonymous=True)
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    rospy.loginfo("joint_state_bridge: connecting to CAN bus...")

    try:
        bus = can.interface.Bus(
            interface=CAN_BUSTYPE,
            channel=CAN_CHANNEL,
            bitrate=CAN_BITRATE
        )
    except Exception as e:
        rospy.logerr(f"Failed to open CAN bus: {e}")
        rospy.logerr("Make sure no other process (slcand, ui.py) is holding /dev/canable")
        return

    rospy.loginfo("joint_state_bridge: CAN bus open. Listening for motor commands...")
    rospy.loginfo("joint_state_bridge: publishing to /joint_states → RViz will update live")

    last_publish = time.time()

    try:
        while not rospy.is_shutdown():
            # Non-blocking receive — timeout 50ms so we publish at ~20Hz
            msg = bus.recv(timeout=PUBLISH_RATE)

            if msg is not None:
                joint_idx = JOINT_CAN_IDS.get(msg.arbitration_id)
                if joint_idx is not None:
                    steps = decode_steps_from_can(msg)
                    if steps is not None:
                        joint_steps[joint_idx]  = steps
                        joint_angles[joint_idx] = steps_to_radians(steps, joint_idx)
                        rospy.logdebug(
                            f"J{joint_idx+1}: {steps} steps → "
                            f"{math.degrees(joint_angles[joint_idx]):.2f}°"
                        )

            # Publish at fixed rate regardless of CAN activity
            now = time.time()
            if now - last_publish >= PUBLISH_RATE:
                js = JointState()
                js.header = Header()
                js.header.stamp = rospy.Time.now()
                js.name     = JOINT_NAMES
                js.position = list(joint_angles)
                js.velocity = [0.0] * 8
                js.effort   = [0.0] * 8
                pub.publish(js)
                last_publish = now

    except KeyboardInterrupt:
        rospy.loginfo("joint_state_bridge: shutting down.")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
