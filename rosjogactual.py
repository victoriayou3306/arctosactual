#!/usr/bin/env python3
import can
import math
import rospy
import os
import re
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import threading

# ─── CONFIG ──────────────────────────────────────────────────────────────────
CAN_INTERFACE = "socketcan"
CAN_CHANNEL   = "can0"
CAN_BITRATE   = 500000

GEAR_RATIOS = [6.75, 75, 75, 24, 33.91, 33.91]
INVERT_DIRECTION = [True, True, False, False, False, False]
DEFAULT_SPEED = 800
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

TAP_FILE = "testing.tap"
MOVE_DELAY = 2.0  # Adjust based on how far your moves are

# ─── STATE ───────────────────────────────────────────────────────────────────
current_angles  = [0.0] * 6
initial_pos     = [0.0] * 6
_bus = None

# ─── HELPERS ─────────────────────────────────────────────────────────────────
def calculate_crc(data_bytes):
    return sum(data_bytes) & 0xFF

def angle_to_can_message(axis_id, speed, angle_rad, gear_ratio):
    idx = axis_id - 1
    angle_deg = math.degrees(angle_rad)
    if INVERT_DIRECTION[idx]:
        angle_deg = -angle_deg
   
    # G90 Logic: We calculate relative step from where the motor last stopped
    rel_position = int((angle_deg * gear_ratio - initial_pos[idx]) * 100)
    initial_pos[idx] = angle_deg * gear_ratio
   
    can_id = format(axis_id, '02X')
    speed_hex = format(speed, '04X')
    rel_position_hex = format(rel_position & 0xFFFFFF, '06X')
    return can_id + 'F5' + speed_hex + '02' + rel_position_hex

def send_can_frames(angles_rad):
    global _bus
    if _bus is None: return
    try:
        for axis_id in range(1, 7):
            idx = axis_id - 1
            hex_msg = angle_to_can_message(axis_id, DEFAULT_SPEED, angles_rad[idx], GEAR_RATIOS[idx])
            rospy.loginfo(f"Sending axis {axis_id}: {hex_msg}")
            data_bytes = [int(hex_msg[i:i+2], 16) for i in range(0, len(hex_msg), 2)]
            crc = calculate_crc(data_bytes)
            full_hex = hex_msg + format(crc, '02X')
            all_bytes = [int(full_hex[i:i+2], 16) for i in range(0, len(full_hex), 2)]
            msg = can.Message(arbitration_id=all_bytes[0], data=all_bytes[1:], is_extended_id=False)
            _bus.send(msg)
    except Exception as e:
        rospy.logerr(f"CAN error: {e}")


_js_lock = threading.Lock()
_last_js_stamp = rospy.Time(0)

def publish_joint_states(pub, angles_rad):
    global _last_js_stamp
    with _js_lock:
        js = JointState()
        js.header = Header()
        now = rospy.Time.now()
        
        # Ensure stamp always moves forward
        if now <= _last_js_stamp:
            now = _last_js_stamp + rospy.Duration(0, 1000000)  # +1ms
        _last_js_stamp = now
        
        js.header.stamp = now
        js.name = JOINT_NAMES
        js.position = list(angles_rad)
        pub.publish(js)

def parse_gcode_line(line):
    """Extracts values. If a coordinate is missing, it keeps the current angle."""
    def get_val(char, string, default):
        match = re.search(rf"{char}([-+]?\d*\.\d+|\d+)", string)
        return float(match.group(1)) if match else default
   
    # We pass the current_angles as defaults to handle lines that only move one axis
    vals = [
        get_val('X', line, math.degrees(current_angles[0])),
        get_val('Y', line, math.degrees(current_angles[1])),
        get_val('Z', line, math.degrees(current_angles[2])),
        get_val('A', line, math.degrees(current_angles[3])),
        get_val('B', line, math.degrees(current_angles[4])),
        get_val('C', line, math.degrees(current_angles[5]))
    ]
    return [math.radians(v) for v in vals]

# ─── COMMAND HANDLER ─────────────────────────────────────────────────────────
def ui_command_callback(msg, pub):
    global initial_pos, current_angles
    data = msg.data.strip()

    if data == "set_origin":
        rospy.loginfo("G90 ORIGIN ESTABLISHED AT CURRENT POSE")
        for i in range(6):
            current_angles[i] = 0.0
            initial_pos[i] = 0.0
        publish_joint_states(pub, current_angles)

    elif data.startswith("go_to_joint_state"):
        parts = data.split(',')
        if len(parts) >= 7:
            angles = [float(parts[i+1]) for i in range(6)]
            for i in range(6): current_angles[i] = angles[i]
            send_can_frames(angles)
            publish_joint_states(pub, angles)

# ─── MAIN LOOP ───────────────────────────────────────────────────────────────
def main():
    global _bus
    rospy.init_node("arctos_gcode_driver", anonymous=True)

    try:
        _bus = can.interface.Bus(interface=CAN_INTERFACE, channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
        rospy.loginfo(f"CAN bus active. Interface: {CAN_INTERFACE}")
    except Exception as e:
        rospy.logerr(f"CRITICAL: CAN bus failed: {e}")

    js_pub = rospy.Publisher("/joint_states", JointState, queue_size=100)
    rospy.Subscriber("/ui_command", String, lambda msg: ui_command_callback(msg, js_pub))

    rospy.loginfo(f"Watching for {TAP_FILE} (G90 Absolute Mode)...")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if os.path.exists(TAP_FILE):
            rospy.loginfo(f"File {TAP_FILE} detected. Executing G90 sequence.")
           
            # Sync start position
            ui_command_callback(String("set_origin"), js_pub)
            rospy.sleep(1.0)

            with open(TAP_FILE, 'r') as f:
                for line in f:
                    line = line.strip().upper()
                    if 'G1' in line or 'G0' in line:
                        target_radians = parse_gcode_line(line)
                       
                        # Execute movement
                        send_can_frames(target_radians)
                        for i in range(6): current_angles[i] = target_radians[i]
                        publish_joint_states(js_pub, target_radians)
                       
                        rospy.loginfo(f"Moved to: {line}")
                        rospy.sleep(MOVE_DELAY)
           
            os.rename(TAP_FILE, TAP_FILE + ".done")
            rospy.loginfo("Sequence complete. File renamed.")
       
        rate.sleep()

    if _bus: _bus.shutdown()

if __name__ == "__main__":
    main()
