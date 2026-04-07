#!/usr/bin/env python3
"""
roscanvel.py — MKS Servo CAN bridge for Arctos arm (F4 relative motion)
----------------------------------------------------------------------
Subscribes to /move_group/fake_controller_joint_states and sends
F4 (relative motion by axis) commands to MKS servo drivers via CAN.

Velocity sync: All joints are velocity-scaled so they finish at the
same time. The joint requiring the highest motor RPM is capped at
MAX_RPM (3000 for SR_vFOC), and all others are scaled down by the
same factor to maintain synchronization. Joints that would fall below
MIN_RPM are boosted (along with all other joints proportionally) to
prevent motor stalling at very low speeds.

Ack waiting: After sending all 6 F4 commands, the bridge waits for
each motor to confirm completion (status=0x02) before accepting the
next move. A dedicated CAN listener thread routes acks into per-motor
queues so nothing is missed.

Move time is latched when new motion is first detected, not when
commands are sent. This prevents a race condition where the trajectory
script overwrites /arctos/move_time for the next waypoint before
roscan has finished processing the current one.

Launch sequence:
  cd ~/catkin_ws/devel && source setup.bash
  roslaunch arctos_config demo.launch
  python3 ~/arctosgui/roscanvel.py
  rosrun moveo_moveit interface.py
  rosrun moveo_moveit transform.py
  python3 ~/arctosgui/arctos_trajectory.py
"""

import math
import threading
import queue
import time
import can
import rospy
from sensor_msgs.msg import JointState

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

# SR_vFOC absolute maximum is 3000 RPM. Any value above this gets clamped
# by the motor firmware, breaking synchronization between joints.
MAX_RPM = 3000

# Minimum reliable motor RPM. Below this, MKS servos may stall or stutter
# because the internal PID can't maintain stable rotation. If any joint
# would fall below this, ALL joints are scaled up proportionally so the
# slowest joint runs at exactly MIN_RPM. The move completes faster than
# requested, but stays synchronized.
MIN_RPM = 50

# Default move time in seconds. All joints are velocity-scaled so that the
# joint requiring the highest motor RPM finishes in this time (or slower if
# it would exceed MAX_RPM). Override live via: rosparam set /arctos/move_time <sec>
DEFAULT_MOVE_TIME = 7.0

MOTOR_ACC = 130  # 0-255

# How long (seconds) with no new joint state before we consider the
# trajectory settled and send the accumulated delta as one command.
SETTLE_TIME = 0.005

# How long to wait for a motor ack before giving up and moving on.
# Set generously -- a slow move at low RPM may take several seconds.
ACK_TIMEOUT = 15.0  # seconds

# Minimum total angle change (radians) to bother sending a command.
MIN_DELTA_RAD = 0.01

# ── STATE ─────────────────────────────────────────────────────────────────────

bus = None

# Per-motor ack queues: ack_queues[i] receives status bytes from motor CAN_IDS[i]
# Queue items are integer status bytes (0x01=starting, 0x02=done, 0x00=fail)
ack_queues = [queue.Queue() for _ in range(6)]

# Map from CAN ID -> joint index for fast ack routing
can_id_to_idx = {CAN_IDS[i]: i for i in range(6)}

# The joint angles we last actually sent to the motors
last_sent_angles = [0.0] * 6

# The latest angles received from MoveIt
latest_angles    = None
last_recv_time   = None
angles_lock      = threading.Lock()

# Mutex so only one move sequence runs at a time
move_lock = threading.Lock()

# ── CAN LISTENER THREAD ───────────────────────────────────────────────────────

def can_listener_loop():
    """
    Dedicated thread that reads ALL incoming CAN frames continuously.
    Routes F4 ack frames into the appropriate per-motor queue.
    Never holds any lock -- completely non-blocking for the sender.

    MKS F4 response format:
      arbitration_id = motor CAN ID
      data[0] = 0xF4  (echo of command code)
      data[1] = 0x01  run starting
               0x02  run complete  <-- this is the "done" signal
               0x00  run fail
               0x03  end limit stopped
    """
    rospy.loginfo("CAN listener thread started")
    while not rospy.is_shutdown():
        try:
            frame = bus.recv(timeout=0.1)
            if frame is None:
                continue

            can_id = frame.arbitration_id
            if can_id not in can_id_to_idx:
                continue

            if len(frame.data) < 2:
                continue

            if frame.data[0] == 0xF4:
                status = frame.data[1]
                idx = can_id_to_idx[can_id]
                ack_queues[idx].put(status)
                rospy.logdebug(
                    f"ACK J{idx+1} (ID {can_id}): status=0x{status:02X}"
                )
        except Exception as e:
            if not rospy.is_shutdown():
                rospy.logerr(f"CAN listener error: {e}")


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


def compute_joint_velocities(deltas_rad, move_time):
    """
    Compute per-joint motor RPM so all joints finish in the same time.

    Then two corrections are applied in order:
      1. If any joint exceeds MAX_RPM, scale ALL joints down so the
         bottleneck runs at exactly MAX_RPM.
      2. If any active joint falls below MIN_RPM, scale ALL joints up
         so the slowest joint runs at exactly MIN_RPM.

    Both corrections preserve the ratio between joints, so they all
    still finish at the same time.

    Returns a list of 6 integer RPM values (0 for joints below MIN_DELTA_RAD).
    """
    raw_rpms = []
    for i in range(6):
        delta_deg = abs(math.degrees(deltas_rad[i]))
        motor_revs = delta_deg / 360.0 * GEAR_RATIOS[i]
        rpm = motor_revs / (move_time / 60.0)
        raw_rpms.append(rpm)

    # Cap at MAX_RPM
    max_raw = max(raw_rpms) if max(raw_rpms) > 0 else 1.0
    scale_down = min(1.0, MAX_RPM / max_raw)
    scaled_rpms = [rpm * scale_down for rpm in raw_rpms]

    # Build final list, zeroing out joints below MIN_DELTA_RAD
    final_rpms = []
    for i, rpm in enumerate(scaled_rpms):
        if abs(deltas_rad[i]) < MIN_DELTA_RAD:
            final_rpms.append(0)
        else:
            final_rpms.append(max(1, int(rpm)))

    # Boost if any active joint is below MIN_RPM
    active_rpms = [r for r in final_rpms if r > 0]
    scale_up = 1.0
    if active_rpms:
        min_active = min(active_rpms)
        if min_active < MIN_RPM:
            scale_up = MIN_RPM / min_active
            max_active = max(active_rpms)
            if max_active * scale_up > MAX_RPM:
                scale_up = MAX_RPM / max_active
            final_rpms = [
                max(1, int(r * scale_up)) if r > 0 else 0
                for r in final_rpms
            ]

    rospy.loginfo(
        f"Velocity plan: move_time={move_time:.4f}s  "
        f"max_raw={max_raw:.0f}  scale_down={scale_down:.3f}  "
        f"scale_up={scale_up:.3f}  rpms={final_rpms}"
    )
    return final_rpms


def send_f4(joint_idx, delta_rad, motor_rpm):
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

    rel_axis = int(delta_deg / 360.0 * ENCODER_CPR * ratio)
    if rel_axis == 0:
        return False

    speed_hi = (motor_rpm >> 8) & 0xFF
    speed_lo =  motor_rpm       & 0xFF
    rel_24   = rel_axis & 0xFFFFFF
    axis_b0  = (rel_24 >> 16) & 0xFF
    axis_b1  = (rel_24 >>  8) & 0xFF
    axis_b2  =  rel_24        & 0xFF

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
            f"delta={delta_deg:+.2f} deg  counts={rel_axis}  "
            f"rpm={motor_rpm}  data={[hex(b) for b in data]}"
        )
        return True
    except Exception as e:
        rospy.logerr(f"CAN send error joint {joint_idx+1}: {e}")
        return False


def wait_for_ack(joint_idx, timeout=ACK_TIMEOUT):
    """
    Block until the motor sends a completion ack (status=0x02),
    or until timeout. Drains 0x01 (starting) frames along the way.
    Returns True on success, False on timeout or error.
    """
    can_id   = CAN_IDS[joint_idx]
    q        = ack_queues[joint_idx]
    deadline = time.time() + timeout

    while time.time() < deadline:
        remaining = deadline - time.time()
        try:
            status = q.get(timeout=min(remaining, 0.1))
        except queue.Empty:
            continue

        if status == 0x02:
            rospy.loginfo(f"ACK done  J{joint_idx+1} (ID {can_id})")
            return True
        elif status == 0x00:
            rospy.logwarn(f"ACK fail  J{joint_idx+1} (ID {can_id}) -- motor reported error")
            return False
        elif status == 0x03:
            rospy.logwarn(f"ACK limit J{joint_idx+1} (ID {can_id}) -- end limit triggered")
            return False
        # status == 0x01: run starting, keep waiting

    rospy.logwarn(
        f"ACK timeout J{joint_idx+1} (ID {can_id}) after {timeout}s -- "
        f"continuing anyway. Check CanRSP is enabled on motor."
    )
    return False


def clear_ack_queues():
    """Drain all ack queues before a new move to avoid stale responses."""
    for q in ack_queues:
        while not q.empty():
            try:
                q.get_nowait()
            except queue.Empty:
                break


# ── SUBSCRIBER CALLBACK ───────────────────────────────────────────────────────

def joint_state_callback(msg):
    """
    Store the latest joint state and timestamp every time MoveIt publishes.
    Does not send anything -- the sender thread decides when to fire.
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
    2. Latches move_time at the moment new motion is first detected.
    3. Computes normalized velocities so all joints finish together.
    4. Sends ONE F4 command per joint for the full accumulated delta.
    5. Waits for completion ack from each motor before accepting next move.
    6. Motors run in parallel physically -- acks are just collected in order.
    """
    global last_sent_angles

    pending = False
    latched_move_time = DEFAULT_MOVE_TIME

    while not rospy.is_shutdown():
        time.sleep(0.02)  # 50 Hz check loop

        with angles_lock:
            target    = list(latest_angles) if latest_angles is not None else None
            recv_time = last_recv_time

        if target is None:
            continue

        mixed_target = apply_differential_mix(target)
        mixed_last   = apply_differential_mix(last_sent_angles)
        deltas = [mixed_target[i] - mixed_last[i] for i in range(6)]

        if any(abs(d) >= MIN_DELTA_RAD for d in deltas):
            if not pending:
                latched_move_time = rospy.get_param(
                    "/arctos/move_time", default=DEFAULT_MOVE_TIME
                )
                rospy.loginfo(
                    f"New motion detected -- latched move_time={latched_move_time:.4f}s"
                )
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

            # Compute synchronized velocities
            joint_rpms = compute_joint_velocities(deltas, latched_move_time)

            # Send all joints simultaneously
            joints_sent = []
            for joint_idx in range(6):
                sent = send_f4(joint_idx, deltas[joint_idx], joint_rpms[joint_idx])
                if sent:
                    joints_sent.append(joint_idx)

            if not joints_sent:
                rospy.loginfo("No joints needed moving (all deltas below threshold)")
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
                rospy.loginfo("All motors confirmed complete")
            else:
                rospy.logwarn(
                    "Some motors timed out or errored -- "
                    "position tracking may have drifted. "
                    "Check CanRSP=Enable on all motors."
                )

            last_sent_angles = list(target)
            pending = False

        finally:
            move_lock.release()


# ── MAIN ──────────────────────────────────────────────────────────────────────

def main():
    global bus

    rospy.init_node("arctos_can_bridge")
    rospy.loginfo("=" * 50)
    rospy.loginfo("Arctos CAN bridge -- settle + velocity-sync + ack-wait mode")
    rospy.loginfo(f"Interface   : {CAN_CHANNEL}")
    rospy.loginfo(f"CAN IDs     : {CAN_IDS}")
    rospy.loginfo(f"CPR         : {ENCODER_CPR}")
    rospy.loginfo(f"Max RPM     : {MAX_RPM}")
    rospy.loginfo(f"Min RPM     : {MIN_RPM}")
    rospy.loginfo(f"Move time   : {DEFAULT_MOVE_TIME}s (override via /arctos/move_time)")
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

    # Start CAN listener before subscribing so no acks are missed
    listener = threading.Thread(target=can_listener_loop, daemon=True)
    listener.start()

    rospy.Subscriber(
        "/move_group/fake_controller_joint_states",
        JointState,
        joint_state_callback,
        queue_size=50
    )

    rospy.loginfo("Subscribed to /move_group/fake_controller_joint_states")
    rospy.loginfo("Ready -- waiting for trajectory to settle before sending.")

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
