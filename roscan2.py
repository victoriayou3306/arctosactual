#!/usr/bin/env python3
"""
roscan2.py — MKS Servo CAN bridge for Arctos arm (F4 relative motion)
----------------------------------------------------------------------
Subscribes to /move_group/fake_controller_joint_states and sends
F4 (relative motion by axis) commands to MKS servo drivers via CAN.

Key fix: Instead of rate-limiting or ack-waiting, this version detects
when the trajectory has SETTLED (stopped changing) and then sends ONE
F4 command per joint for the entire accumulated delta. This means the
motor gets a single clean move command rather than being interrupted
by a stream of tiny incremental commands.

Launch sequence:
  cd ~/catkin_ws/devel && source setup.bash
  roslaunch arctos_config demo.launch
  python3 ~/arctosgui/roscan2.py
  rosrun moveo_moveit interface.py
  rosrun moveo_moveit transform.py
  python3 ~/arctosgui/arctos_trajectory.py
"""

import math
import threading
import time
import can
import rospy
from sensor_msgs.msg import JointState

# ── CONFIGURATION ─────────────────────────────────────────────────────────────

CAN_INTERFACE = "slcan"
CAN_CHANNEL   = "/dev/ttyACM0"
CAN_BITRATE   = 500000

CAN_IDS          = [7, 8, 9, 10, 11, 12]
#GEAR_RATIOS      = [6.75, 75, 75, 24, 33.91, 33.91]
GEAR_RATIOS = [13.5, 150, 150, 48, 67.82, 67.82]
INVERT_DIRECTION = [True, False, True, False, True, False]

# Per the MKS manual section 6.7: F4 relAxis is in encoder axis units.
# One full motor revolution = 0x4000 = 16384 counts (command "31").
ENCODER_CPR = 16384

MOTOR_SPEED = 1000   # RPM
MOTOR_ACC   = 20     # 0-255

# How long (seconds) with no new joint state before we consider the
# trajectory settled and send the accumulated delta as one command.
# MoveIt trajectories typically finish publishing within 0.5-1s.
# Raise this if commands fire too early; lower for faster response.
SETTLE_TIME = 0.15

# Minimum total angle change (radians) to bother sending a command.
MIN_DELTA_RAD = 0.01

# ── STATE ─────────────────────────────────────────────────────────────────────

bus = None

# The joint angles we last actually sent to the motors
last_sent_angles = [0.0] * 6

# The latest angles received from MoveIt
latest_angles    = None
last_recv_time   = None
angles_lock      = threading.Lock()

# ── HELPERS ───────────────────────────────────────────────────────────────────

def calculate_crc(can_id, data_bytes):
    return (can_id + sum(data_bytes)) & 0xFF


def apply_differential_mix(angles_rad):
    """
    Bevel-gear differential for joints 5/6.
    B = joint5, C = joint6
    M5 = (C + B) / 2
    M6 = (C - B) / 2
    """
    mixed = list(angles_rad)
    c = angles_rad[4]
    b = angles_rad[5]
    mixed[4] = (c + b) / 2.0
    mixed[5] = (c - b) / 2.0
    return mixed


def send_f4(joint_idx, delta_rad):
    """
    Send a single F4 relative-move command for one joint.
    delta_rad is the total move in output-shaft radians.
    Returns True if a command was sent, False if delta was too small.
    """
    if abs(delta_rad) < MIN_DELTA_RAD:
        return False

    can_id = CAN_IDS[joint_idx]
    ratio  = GEAR_RATIOS[joint_idx]

    delta_deg = math.degrees(delta_rad)
    if INVERT_DIRECTION[joint_idx]:
        delta_deg = -delta_deg

    # F4 relAxis = encoder counts on motor shaft
    # motor shaft revolutions = output_revolutions * gear_ratio
    print(delta_deg*ratio)
    rel_axis = int(delta_deg / 360.0 * ENCODER_CPR * ratio)

    if rel_axis == 0:
        return False

    speed_hi    = (MOTOR_SPEED >> 8) & 0xFF
    speed_lo    =  MOTOR_SPEED       & 0xFF
    rel_24      = rel_axis & 0xFFFFFF
    axis_b0     = (rel_24 >> 16) & 0xFF
    axis_b1     = (rel_24 >>  8) & 0xFF
    axis_b2     =  rel_24        & 0xFF

    data = [0xF4, speed_hi, speed_lo, MOTOR_ACC, axis_b0, axis_b1, axis_b2]
    crc  = calculate_crc(can_id, data)
    data.append(crc)

    try:
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False
        )
        bus.send(msg)
        rospy.loginfo(
            f"CAN J{joint_idx+1} (ID {can_id}): "
            f"delta={delta_deg:+.2f}° "
            f"counts={rel_axis} "
            f"data={[hex(b) for b in data]}"
        )
        return True
    except Exception as e:
        rospy.logerr(f"CAN send error joint {joint_idx+1}: {e}")
        return False


# ── SUBSCRIBER CALLBACK ───────────────────────────────────────────────────────

def joint_state_callback(msg):
    """
    Store the latest joint state and timestamp every time MoveIt publishes.
    Does not send anything — the sender thread decides when to fire.
    """
    global latest_angles, last_recv_time

    joint_map = {f"joint{j+1}": j for j in range(6)}
    new_angles = list(last_sent_angles)
    for i, name in enumerate(msg.name):
        if name in joint_map:
            new_angles[joint_map[name]] = msg.position[i]

    with angles_lock:
        latest_angles  = new_angles
        last_recv_time = time.time()


# ── SENDER THREAD ─────────────────────────────────────────────────────────────

def can_sender_loop():
    """
    Watches for the trajectory to settle (no new messages for SETTLE_TIME),
    then sends ONE F4 command per joint covering the full accumulated delta
    since the last send. This avoids the jitter caused by sending a command
    for every intermediate trajectory point.
    """
    global last_sent_angles

    pending = False  # True when we have unsent data and are waiting to settle

    while not rospy.is_shutdown():
        time.sleep(0.02)  # 50 Hz check loop

        with angles_lock:
            target    = list(latest_angles) if latest_angles is not None else None
            recv_time = last_recv_time

        if target is None:
            continue

        # Check if there's a meaningful difference from what we last sent
        mixed_target = apply_differential_mix(target)
        mixed_last   = apply_differential_mix(last_sent_angles)
        deltas = [mixed_target[i] - mixed_last[i] for i in range(6)]

        if any(abs(d) >= MIN_DELTA_RAD for d in deltas):
            pending = True

        if not pending:
            continue

        # Wait for trajectory to settle
        elapsed = time.time() - recv_time
        if elapsed < SETTLE_TIME:
            continue

        # Trajectory has settled — send the full accumulated delta
        rospy.loginfo(
            f"Trajectory settled ({elapsed:.2f}s since last msg). "
            f"Sending full delta to motors."
        )

        for joint_idx in range(6):
            send_f4(joint_idx, deltas[joint_idx])

        last_sent_angles = list(target)
        pending = False


# ── MAIN ──────────────────────────────────────────────────────────────────────

def main():
    global bus

    rospy.init_node("arctos_can_bridge")
    rospy.loginfo("=" * 50)
    rospy.loginfo("Arctos CAN bridge — settle-and-send mode")
    rospy.loginfo(f"Interface   : {CAN_CHANNEL}")
    rospy.loginfo(f"CAN IDs     : {CAN_IDS}")
    rospy.loginfo(f"CPR         : {ENCODER_CPR}")
    rospy.loginfo(f"Speed       : {MOTOR_SPEED} RPM")
    rospy.loginfo(f"Accel       : {MOTOR_ACC}")
    rospy.loginfo(f"Settle time : {SETTLE_TIME}s")
    rospy.loginfo("=" * 50)

    try:
        bus = can.interface.Bus(
            interface=CAN_INTERFACE,
            channel=CAN_CHANNEL,
            bitrate=CAN_BITRATE
        )
        rospy.loginfo(f"CAN bus open on {CAN_CHANNEL}")
    except Exception as e:
        rospy.logerr(f"Failed to open CAN bus: {e}")
        return

    rospy.Subscriber(
        "/move_group/fake_controller_joint_states",
        JointState,
        joint_state_callback,
        queue_size=50
    )

    rospy.loginfo("Subscribed to /move_group/fake_controller_joint_states")
    rospy.loginfo("Ready — waiting for trajectory to settle before sending.")

    sender = threading.Thread(target=can_sender_loop, daemon=True)
    sender.start()

    rospy.spin()
    bus.shutdown()
    rospy.loginfo("CAN bus closed.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nStopped by user.")
        if bus:
            bus.shutdown()
