#!/usr/bin/env python3
"""
rosjog.py (Updated)
=========
Subscribes to /ui_command from the Arctos GUI or a TAP loader script.
Commands:
  1. "go_to_joint_state,j1,j2,j3,j4,j5,j6" -> Moves arm via CAN
  2. "set_origin" -> Resets software position to Zero at current physical spot
  3. Publishes to /joint_states for RViz animation
"""

import can
import math
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# ─── CONFIG ──────────────────────────────────────────────────────────────────

CAN_INTERFACE = "slcan"
CAN_CHANNEL   = "/dev/ttyACM0"
CAN_BITRATE   = 500000

# Gear ratios per joint
GEAR_RATIOS = [6.75, 75, 75, 24, 33.91, 33.91]

# Direction inversion per joint
INVERT_DIRECTION = [True, True, False, False, False, False]

# Speed for CAN moves
DEFAULT_SPEED = 800

# Joint names must match URDF exactly
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# ─── STATE ───────────────────────────────────────────────────────────────────

current_angles  = [0.0] * 6   # radians
initial_pos     = [0.0] * 6   # tracks accumulated gear-ratio position (degrees * ratio)
last_pos        = [0.0] * 6
_bus = None

# ─── HELPERS ─────────────────────────────────────────────────────────────────

def calculate_crc(data_bytes):
    return sum(data_bytes) & 0xFF

def angle_to_can_message(axis_id, speed, angle_rad, gear_ratio):
    """
    Convert a joint angle (radians) to an Arctos CAN hex string.
    """
    idx = axis_id - 1

    # Convert radians → degrees for Arctos protocol
    angle_deg = math.degrees(angle_rad)

    # Apply direction inversion
    if INVERT_DIRECTION[idx]:
        angle_deg = -angle_deg

    # Calculate relative position scaled x100
    # Note: initial_pos is updated after the move to track where the motor is 'now'
    rel_position = int((angle_deg * gear_ratio - initial_pos[idx]) * 100)

    # Update tracking for the NEXT move
    initial_pos[idx] = angle_deg * gear_ratio

    can_id           = format(axis_id, '02X')
    speed_hex        = format(speed, '04X')
    rel_position_hex = format(rel_position & 0xFFFFFF, '06X')

    return can_id + 'F5' + speed_hex + '02' + rel_position_hex

def send_can_frames(angles_rad):
    global _bus
    if _bus is None:
        rospy.logerr("CAN bus not initialized")
        return
    try:
        for axis_id in range(1, 7):
            idx       = axis_id - 1
            hex_msg   = angle_to_can_message(axis_id, DEFAULT_SPEED, angles_rad[idx], GEAR_RATIOS[idx])
            data_bytes = [int(hex_msg[i:i+2], 16) for i in range(0, len(hex_msg), 2)]
            crc        = calculate_crc(data_bytes)
            full_hex   = hex_msg + format(crc, '02X')
            all_bytes  = [int(full_hex[i:i+2], 16) for i in range(0, len(full_hex), 2)]
            msg = can.Message(
                arbitration_id=all_bytes[0],
                data=all_bytes[1:],
                is_extended_id=False
            )
            _bus.send(msg)
            # rospy.loginfo(f"CAN sent J{axis_id}")
    except Exception as e:
        rospy.logerr(f"CAN send error: {e}")

def publish_joint_states(pub, angles_rad):
    """Publish joint angles to /joint_states for RViz."""
    js              = JointState()
    js.header       = Header()
    js.header.stamp = rospy.Time.now()
    js.name         = JOINT_NAMES
    js.position     = list(angles_rad)
    js.velocity     = [0.0] * 6
    js.effort       = [0.0] * 6
    pub.publish(js)

# ─── COMMAND HANDLER ─────────────────────────────────────────────────────────

def ui_command_callback(msg, pub):
    """Handle commands published to /ui_command."""
    global initial_pos, current_angles
    data = msg.data.strip()

    # 1. SET ORIGIN: Resets software zero to current physical position
    if data == "set_origin":
        rospy.loginfo("SET_ORIGIN: Synchronizing software zero to current pose.")
        for i in range(6):
            current_angles[i] = 0.0
            # We reset initial_pos to 0 so the next relative CAN calc starts from 0
            initial_pos[i] = 0.0 
        
        publish_joint_states(pub, current_angles)
        rospy.loginfo("Origin reset complete. RViz updated.")

    # 2. MOVE TO STATE: Standard movement command
    elif data.startswith("go_to_joint_state"):
        parts = data.split(',')
        if len(parts) >= 7:
            try:
                angles = [float(parts[i+1]) for i in range(6)]
                
                # Update local state
                for i in range(6):
                    current_angles[i] = angles[i]

                # Send CAN frames to hardware
                send_can_frames(angles)

                # Update RViz
                publish_joint_states(pub, angles)

            except ValueError as e:
                rospy.logerr(f"Bad joint state values: {e}")

    elif data == "open_gripper":
        rospy.loginfo("Gripper: OPEN")

    elif data == "close_gripper":
        rospy.loginfo("Gripper: CLOSE")

# ─── MAIN ────────────────────────────────────────────────────────────────────

def main():
    global _bus
    rospy.init_node("arctos_rosjog", anonymous=True)

    try:
        _bus = can.interface.Bus(
            interface=CAN_INTERFACE,
            channel=CAN_CHANNEL,
            bitrate=CAN_BITRATE
        )
        rospy.loginfo(f"CAN bus open on {CAN_CHANNEL}")
    except Exception as e:
        rospy.logerr(f"Failed to open CAN bus: {e}")

    js_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rospy.sleep(0.5)

    rospy.Subscriber(
        "/ui_command",
        String,
        lambda msg: ui_command_callback(msg, js_pub)
    )

    rospy.loginfo("rosjog ready — Use 'set_origin' or 'go_to_joint_state'")
    rospy.spin()

    if _bus:
        _bus.shutdown()

if __name__ == "__main__":
    main()
