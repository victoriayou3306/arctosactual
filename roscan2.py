#!/usr/bin/env python3
"""
roscan2.py — MKS Servo CAN bridge for Arctos arm (F4 relative motion)
----------------------------------------------------------------------
Subscribes to /move_group/fake_controller_joint_states and sends
F4 (relative motion by axis) commands to MKS servo drivers via CAN.

Architecture:
  - CAN listener thread: reads all incoming frames continuously,
    routes F4 completion acks into per-motor queues.
  - Settle detector: waits for trajectory to stop publishing, then
    sends ONE F4 per joint for the full accumulated delta.
  - Ack waiter: after sending all 6 joints, waits for each motor to
    confirm completion (status=0x02) before publishing move_complete.
    Motors run in parallel physically; we just collect confirmations.

After all motors confirm, publishes True on /arctos/move_complete so
arctos_trajectory.py can safely send the next waypoint.

Launch sequence:
  cd ~/catkin_ws/devel && source setup.bash
  roslaunch arctos_config demo.launch
  python3 ~/arctosgui/roscan2.py
  rosrun moveo_moveit interface.py
  rosrun moveo_moveit transform.py
  python3 arctos_trajectory.py
"""

import math
import threading
import queue
import time
import can
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

# ── CONFIGURATION ─────────────────────────────────────────────────────────────

CAN_INTERFACE = "slcan"
CAN_CHANNEL   = "/dev/ttyACM0"
CAN_BITRATE   = 500000

CAN_IDS          = [7, 8, 9, 10, 11, 12]
GEAR_RATIOS      = [13.5, 150, 150, 48, 67.82, 67.82]
INVERT_DIRECTION = [True, False, True, False, True, False]

# Per the MKS manual section 6.7: F4 relAxis is in encoder axis units.
# One full motor revolution = 0x4000 = 16384 counts (command "31").
ENCODER_CPR = 16384

MOTOR_SPEED = 1000   # RPM (0-3000)
MOTOR_ACC   = 20     # 0-255

# How long (seconds) with no new joint state before we consider the
# trajectory settled and send the accumulated delta as one command.
SETTLE_TIME = 0.15

# How long to wait for a motor ack before giving up.
# Set generously: a slow move at low speed may take several seconds.
ACK_TIMEOUT = 15.0   # seconds

# Minimum total angle change (radians) to bother sending a command.
MIN_DELTA_RAD = 0.01

# ── STATE ─────────────────────────────────────────────────────────────────────

bus = None
move_complete_pub = None

# The joint angles we last actually sent to the motors
last_sent_angles = [0.0] * 6

# The latest angles received from MoveIt
latest_angles    = None
last_recv_time   = None
angles_lock      = threading.Lock()

# Prevents overlapping move sequences
move_lock = threading.Lock()

# Per-motor ack queues. The CAN listener thread puts status bytes here.
# Index by joint_idx (0-5).
ack_queues = [queue.Queue() for _ in range(6)]

# Map CAN arbitration IDs back to joint indices for ack routing
CAN_ID_TO_JOINT = {cid: idx for idx, cid in enumerate(CAN_IDS)}

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


def clear_ack_queues():
    """Drain any stale acks from previous moves."""
    for q in ack_queues:
        while not q.empty():
            try:
                q.get_nowait()
            except queue.Empty:
                break


def send_f4(joint_idx, delta_rad):
    """
    Send a single F4 relative-move command for one joint.
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
            f"delta={delta_deg:+.2f} deg "
            f"counts={rel_axis} "
            f"data={[hex(b) for b in data]}"
        )
        return True
    except Exception as e:
        rospy.logerr(f"CAN send error joint {joint_idx+1}: {e}")
        return False


def wait_for_ack(joint_idx):
    """
    Block until motor confirms F4 completion (status 0x02) or timeout.

    MKS F4 response frame: [0xF4, status]
      status = 0x00: command failed / motor error
      status = 0x01: command running (intermediate, keep waiting)
      status = 0x02: command completed successfully
      status = 0x03: end-limit stopped

    Returns True if motor confirmed completion, False on timeout/error.
    """
    can_id = CAN_IDS[joint_idx]
    deadline = time.time() + ACK_TIMEOUT

    while time.time() < deadline:
        try:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            status = ack_queues[joint_idx].get(timeout=min(remaining, 0.5))

            if status == 0x02:
                rospy.loginfo(f"J{joint_idx+1} (ID {can_id}): move complete (0x02)")
                return True
            elif status == 0x01:
                # Still running, keep waiting
                continue
            elif status == 0x00:
                rospy.logwarn(f"J{joint_idx+1} (ID {can_id}): motor error (0x00)")
                return False
            elif status == 0x03:
                rospy.logwarn(f"J{joint_idx+1} (ID {can_id}): end-limit stop (0x03)")
                return False
            else:
                rospy.logwarn(
                    f"J{joint_idx+1} (ID {can_id}): unexpected status 0x{status:02X}"
                )
                continue

        except queue.Empty:
            continue

    rospy.logwarn(
        f"J{joint_idx+1} (ID {can_id}): ack timeout after {ACK_TIMEOUT}s. "
        f"Position may have drifted."
    )
    return False


# ── CAN LISTENER THREAD ──────────────────────────────────────────────────────

def can_listener_loop():
    """
    Continuously reads CAN frames and routes F4 ack responses into
    per-motor queues. Runs as a daemon thread; never blocks the sender.
    """
    while not rospy.is_shutdown():
        try:
            msg = bus.recv(timeout=0.1)
            if msg is None:
                continue

            arb_id = msg.arbitration_id
            data   = msg.data

            # F4 response: [0xF4, status_byte]
            if arb_id in CAN_ID_TO_JOINT and len(data) >= 2 and data[0] == 0xF4:
                joint_idx = CAN_ID_TO_JOINT[arb_id]
                status    = data[1]
                ack_queues[joint_idx].put(status)

        except can.CanError as e:
            rospy.logwarn(f"CAN read error: {e}")
        except Exception as e:
            if not rospy.is_shutdown():
                rospy.logwarn(f"CAN listener error: {e}")


# ── SUBSCRIBER CALLBACK ───────────────────────────────────────────────────────

def joint_state_callback(msg):
    """
    Store the latest joint state and timestamp every time MoveIt publishes.
    Does not send anything; the sender thread decides when to fire.
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
    1. Waits for trajectory to settle (SETTLE_TIME with no new messages).
    2. Sends ONE F4 command per joint for the full accumulated delta.
    3. Waits for completion ack from each motor before publishing move_complete.
    4. Motors run in parallel physically; acks are collected sequentially.
    """
    global last_sent_angles

    pending = False

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

        # Only one move sequence at a time
        if not move_lock.acquire(blocking=False):
            continue

        try:
            rospy.loginfo(
                f"Trajectory settled ({elapsed:.2f}s since last msg). "
                f"Sending full delta to motors."
            )

            # Clear stale acks from any previous move
            clear_ack_queues()

            # Send all joints simultaneously
            joints_sent = []
            for joint_idx in range(6):
                sent = send_f4(joint_idx, deltas[joint_idx])
                if sent:
                    joints_sent.append(joint_idx)

            if not joints_sent:
                rospy.loginfo("No joints needed moving (all deltas below threshold).")
                last_sent_angles = list(target)
                pending = False
                continue

            # Wait for all sent joints to complete
            rospy.loginfo(
                f"Waiting for acks from joints: "
                f"{[j+1 for j in joints_sent]}"
            )

            all_ok = True
            for joint_idx in joints_sent:
                ok = wait_for_ack(joint_idx)
                if not ok:
                    all_ok = False

            if all_ok:
                rospy.loginfo("All motors confirmed complete.")
            else:
                rospy.logwarn(
                    "Some motors timed out or errored. "
                    "Position tracking may have drifted."
                )

            last_sent_angles = list(target)
            pending = False

            # Notify trajectory runner that this move is physically done
            if move_complete_pub is not None:
                move_complete_pub.publish(Bool(data=True))
                rospy.loginfo("Published move_complete.")

        finally:
            move_lock.release()


# ── MAIN ──────────────────────────────────────────────────────────────────────

def main():
    global bus, move_complete_pub

    rospy.init_node("arctos_can_bridge")
    rospy.loginfo("=" * 50)
    rospy.loginfo("Arctos CAN bridge (settle + ack-wait + sync)")
    rospy.loginfo(f"Interface   : {CAN_CHANNEL}")
    rospy.loginfo(f"CAN IDs     : {CAN_IDS}")
    rospy.loginfo(f"Gear ratios : {GEAR_RATIOS}")
    rospy.loginfo(f"CPR         : {ENCODER_CPR}")
    rospy.loginfo(f"Speed       : {MOTOR_SPEED} RPM")
    rospy.loginfo(f"Accel       : {MOTOR_ACC}")
    rospy.loginfo(f"Settle time : {SETTLE_TIME}s")
    rospy.loginfo(f"Ack timeout : {ACK_TIMEOUT}s")
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

    # Publisher so arctos_trajectory.py can synchronize
    move_complete_pub = rospy.Publisher(
        "/arctos/move_complete", Bool, queue_size=10
    )

    rospy.Subscriber(
        "/move_group/fake_controller_joint_states",
        JointState,
        joint_state_callback,
        queue_size=50
    )

    rospy.loginfo("Subscribed to /move_group/fake_controller_joint_states")
    rospy.loginfo("Publishing move_complete on /arctos/move_complete")
    rospy.loginfo("Ready.")

    # Start CAN listener (reads ack frames continuously)
    listener = threading.Thread(target=can_listener_loop, daemon=True)
    listener.start()

    # Start sender (settle-detect, send F4s, wait for acks)
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
