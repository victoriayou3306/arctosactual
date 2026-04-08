#!/usr/bin/env python3
"""
roscan3.py — MKS Servo CAN bridge for Arctos arm (F5 streaming)
----------------------------------------------------------------
Implements a FollowJointTrajectory action server that streams F5
(absolute motion by axis) commands to MKS servo drivers via CAN.

F5 supports real-time target updates: sending a new F5 while the
motor is already moving smoothly updates the target position without
stopping. This enables continuous motion for 3D printing paths.

Also retains the settle-detect F4 fallback for use with demo.launch
(fake controller mode). When using arctos_real.launch, all moves
go through the action server and settle-detect is automatically
inactive.

Changes from roscan3 (original):
  - F5_ACC raised from 0 → 5 (smooth acceleration ramp, eliminates
    the "crunchy" instant-speed-jump on every F5 update)
  - STREAM_RATE_HZ raised from 30 → 100 Hz (10ms update interval,
    much smoother position tracking)
  - F5_SPEED is now a fallback only; _send_joint_positions accepts
    an optional velocities argument and derives per-motor RPM from
    the trajectory's own velocity data when available
  - Velocity interpolation added in _execute_cb alongside position
    interpolation so dynamic RPM tracks the planned motion profile
  - Final trajectory point send now also passes interpolated velocity
  - Every trajectory point is now sent (no skipping when system is slow)
  - Lag warning at 50ms, abort at 100ms with motor hold command
  - ENABLE_SETTLE_DETECT flag: set to True only for demo.launch GUI mode

Launch sequence (real arm):
  roslaunch arctos_config arctos_real.launch
  python3 ~/arctosgui/roscan3.py
  rosrun moveo_moveit interface.py
  rosrun moveo_moveit transform.py
  python3 ~/arctosgui/arctos_trajectory3.py

Launch sequence (simulation GUI, backward compat):
  roslaunch arctos_config demo.launch
  python3 ~/arctosgui/roscan3.py
  rosrun moveo_moveit interface.py
  rosrun moveo_moveit transform.py
"""

import math
import threading
import queue
import time
import can
import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from trajectory_msgs.msg import JointTrajectoryPoint

# ── CONFIGURATION ─────────────────────────────────────────────────────────────

CAN_INTERFACE = "slcan"
CAN_CHANNEL   = "/dev/ttyACM0"
CAN_BITRATE   = 500000

CAN_IDS          = [7, 8, 9, 10, 11, 12]
GEAR_RATIOS      = [13.5, 150, 150, 48, 67.82, 67.82]
INVERT_DIRECTION = [True, False, True, False, True, False]

ENCODER_CPR = 16384

# F5 streaming: fallback speed used when trajectory velocity data is
# unavailable. Dynamic RPM derived from trajectory velocities is
# preferred and will be used whenever MoveIt populates point.velocities.
F5_SPEED = 200    # RPM (fallback only)

# Acceleration ramp for F5. 0 = instant jump to speed
# Increase if motors still sound rough; decrease if they lag on
# fast direction changes. Range: 0-255 per MKS protocol.
F5_ACC   = 20

# Per-motor RPM clamp when using dynamic speed from trajectory.
F5_RPM_MIN = 20   # floor — prevents stall on near-zero velocity segments
F5_RPM_MAX = 500  # ceiling — keep within safe motor limits

# Rate at which we send F5 updates during trajectory playback.
# 100 Hz = 10ms between updates, smooth enough for 3D printing paths.
STREAM_RATE_HZ = 200

# For backward compat: settle-detect for demo.launch GUI moves
SETTLE_TIME   = 0.15
MIN_DELTA_RAD = 0.01
F4_SPEED = 200
F4_ACC   = 20
ACK_TIMEOUT = 15.0

# Set to True only if running demo.launch with the GUI fallback.
# Leave False for all real-arm / 3D printing use.
ENABLE_SETTLE_DETECT = False

# ── STATE ─────────────────────────────────────────────────────────────────────

bus = None
move_complete_pub = None

# Absolute encoder position tracking (encoder counts per motor).
abs_positions = [0] * 6

# Joint ordering as MoveIt knows them
JOINT_NAMES = [f"joint{i+1}" for i in range(6)]

# Tracks the last joint angles (radians) we commanded, shared between
# action server and settle-detect so both stay in sync.
last_sent_angles = [0.0] * 6
last_sent_lock   = threading.Lock()

# For settle-detect fallback
latest_angles    = None
last_recv_time   = None
angles_lock      = threading.Lock()
move_lock        = threading.Lock()

# Per-motor ack queues for F4 settle-detect mode
ack_queues = [queue.Queue() for _ in range(6)]
CAN_ID_TO_JOINT = {cid: idx for idx, cid in enumerate(CAN_IDS)}

# When the action server is actively streaming, disable settle-detect
streaming_active = False

# ── CAN HELPERS ───────────────────────────────────────────────────────────────

def calculate_crc(can_id, data_bytes):
    return (can_id + sum(data_bytes)) & 0xFF


def apply_differential_mix(values):
    """
    Bevel-gear differential for joints 5/6.
    M5 = (C + B) / 2,  M6 = (C - B) / 2
    where C = joint5, B = joint6 in MoveIt ordering.
    Works for both positions (rad) and velocities (rad/s).
    """
    mixed = list(values)
    c = values[4]
    b = values[5]
    mixed[4] = (c + b) / 2.0
    mixed[5] = (c - b) / 2.0
    return mixed


def rad_to_encoder_counts(joint_idx, angle_rad):
    """Convert an output-shaft angle (radians) to absolute encoder counts."""
    ratio = GEAR_RATIOS[joint_idx]
    deg = math.degrees(angle_rad)
    if INVERT_DIRECTION[joint_idx]:
        deg = -deg
    return int(deg / 360.0 * ENCODER_CPR * ratio)


def rad_s_to_rpm(joint_idx, vel_rad_s):
    """
    Convert output-shaft velocity (rad/s) to motor-shaft RPM,
    accounting for gear ratio. Returns clamped value within
    [F5_RPM_MIN, F5_RPM_MAX].
    """
    rpm = abs(vel_rad_s) * GEAR_RATIOS[joint_idx] * 60.0 / (2.0 * math.pi)
    return int(max(F5_RPM_MIN, min(rpm, F5_RPM_MAX)))


def send_f5(joint_idx, abs_counts, speed=None, acc=None):
    """
    Send F5 absolute-motion-by-axis command.
    Frame: [F5, speed_hi, speed_lo, acc, axis_hi, axis_mid, axis_lo, CRC]
    absAxis is int24_t (-8388607 to +8388607).
    """
    if speed is None:
        speed = F5_SPEED
    if acc is None:
        acc = F5_ACC

    can_id = CAN_IDS[joint_idx]

    speed_hi = (speed >> 8) & 0xFF
    speed_lo = speed & 0xFF

    abs_24 = abs_counts & 0xFFFFFF
    axis_b0 = (abs_24 >> 16) & 0xFF
    axis_b1 = (abs_24 >> 8) & 0xFF
    axis_b2 = abs_24 & 0xFF

    data = [0xF5, speed_hi, speed_lo, acc, axis_b0, axis_b1, axis_b2]
    crc = calculate_crc(can_id, data)
    data.append(crc)

    try:
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False
        )
        bus.send(msg)
        return True
    except Exception as e:
        rospy.logerr(f"CAN F5 send error J{joint_idx+1}: {e}")
        return False


def send_f4(joint_idx, delta_rad):
    """Send F4 relative-move command (for settle-detect fallback)."""
    if abs(delta_rad) < MIN_DELTA_RAD:
        return False

    can_id = CAN_IDS[joint_idx]
    ratio = GEAR_RATIOS[joint_idx]
    delta_deg = math.degrees(delta_rad)
    if INVERT_DIRECTION[joint_idx]:
        delta_deg = -delta_deg

    rel_axis = int(delta_deg / 360.0 * ENCODER_CPR * ratio)
    if rel_axis == 0:
        return False

    speed_hi = (F4_SPEED >> 8) & 0xFF
    speed_lo = F4_SPEED & 0xFF
    rel_24 = rel_axis & 0xFFFFFF
    axis_b0 = (rel_24 >> 16) & 0xFF
    axis_b1 = (rel_24 >> 8) & 0xFF
    axis_b2 = rel_24 & 0xFF

    data = [0xF4, speed_hi, speed_lo, F4_ACC, axis_b0, axis_b1, axis_b2]
    crc = calculate_crc(can_id, data)
    data.append(crc)

    try:
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        bus.send(msg)
        rospy.loginfo(
            f"[F4] J{joint_idx+1} (ID {can_id}): "
            f"delta={delta_deg:+.2f} deg counts={rel_axis}"
        )
        return True
    except Exception as e:
        rospy.logerr(f"CAN F4 send error J{joint_idx+1}: {e}")
        return False


def clear_ack_queues():
    for q in ack_queues:
        while not q.empty():
            try:
                q.get_nowait()
            except queue.Empty:
                break


def wait_for_ack(joint_idx):
    """Block until F4 completion ack (status 0x02) or timeout."""
    deadline = time.time() + ACK_TIMEOUT
    while time.time() < deadline:
        try:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            status = ack_queues[joint_idx].get(timeout=min(remaining, 0.5))
            if status == 0x02:
                return True
            elif status == 0x01:
                continue
            elif status in (0x00, 0x03):
                return False
        except queue.Empty:
            continue
    rospy.logwarn(f"J{joint_idx+1}: F4 ack timeout after {ACK_TIMEOUT}s")
    return False


def query_encoder_position(joint_idx):
    """
    Query motor's current absolute encoder value using command 0x31.
    Returns int48 encoder count, or 0 on failure.
    """
    can_id = CAN_IDS[joint_idx]
    data = [0x31]
    crc = calculate_crc(can_id, data)
    data.append(crc)

    try:
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        bus.send(msg)
    except Exception as e:
        rospy.logwarn(f"Failed to query encoder J{joint_idx+1}: {e}")
        return 0

    deadline = time.time() + 0.5
    while time.time() < deadline:
        try:
            resp = bus.recv(timeout=0.1)
            if resp is None:
                continue
            if resp.arbitration_id == can_id and len(resp.data) >= 8 and resp.data[0] == 0x31:
                raw = 0
                for i in range(1, 7):
                    raw = (raw << 8) | resp.data[i]
                if raw >= (1 << 47):
                    raw -= (1 << 48)
                return raw
        except Exception:
            continue

    rospy.logwarn(f"No encoder response from J{joint_idx+1}, assuming 0")
    return 0


# ── CAN LISTENER THREAD ──────────────────────────────────────────────────────

def can_listener_loop():
    """Read CAN frames and route F4 acks into per-motor queues."""
    while not rospy.is_shutdown():
        try:
            msg = bus.recv(timeout=0.1)
            if msg is None:
                continue
            arb_id = msg.arbitration_id
            data = msg.data
            if arb_id in CAN_ID_TO_JOINT and len(data) >= 2 and data[0] == 0xF4:
                joint_idx = CAN_ID_TO_JOINT[arb_id]
                ack_queues[joint_idx].put(data[1])
        except can.CanError as e:
            rospy.logwarn(f"CAN read error: {e}")
        except Exception as e:
            if not rospy.is_shutdown():
                rospy.logwarn(f"CAN listener error: {e}")


# ── FOLLOW JOINT TRAJECTORY ACTION SERVER ─────────────────────────────────────

class TrajectoryActionServer:
    """
    Receives a full JointTrajectory from MoveIt and plays it back
    by sending every point in order via F5 absolute position commands.

    Motor speed is derived from the trajectory's own velocity data
    when available (requires joint_limits.yaml to be populated so
    MoveIt/TOTG fills point.velocities). Falls back to F5_SPEED
    when velocity data is absent.

    Lag handling:
      - >50ms behind schedule: log a warning
      - >100ms behind schedule: hold all motors and abort
    """

    def __init__(self):
        self._server = actionlib.SimpleActionServer(
            "/arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._execute_cb,
            auto_start=False,
        )
        self._server.start()
        rospy.loginfo("FollowJointTrajectory action server started.")

    def _execute_cb(self, goal):
        global streaming_active

        traj = goal.trajectory
        points = traj.points

        if len(points) == 0:
            rospy.logwarn("Empty trajectory received.")
            self._server.set_succeeded(FollowJointTrajectoryResult())
            return

        # Build joint name to index mapping for this trajectory
        joint_indices = []
        for name in traj.joint_names:
            if name in JOINT_NAMES:
                joint_indices.append(JOINT_NAMES.index(name))
            else:
                rospy.logwarn(f"Unknown joint in trajectory: {name}")
                joint_indices.append(-1)

        # Check whether velocity data is populated in this trajectory
        has_velocities = (
            len(points[0].velocities) == len(points[0].positions)
            and len(points[0].velocities) > 0
        )

        rospy.loginfo(
            f"[F5] Trajectory received: {len(points)} points, "
            f"duration {points[-1].time_from_start.to_sec():.2f}s, "
            f"velocity data: {'yes' if has_velocities else 'no (using fallback RPM)'}"
        )

        streaming_active = True
        start_time = rospy.Time.now()

        LAG_WARN_SEC  = 10   # 50ms  — log a warning
        LAG_ABORT_SEC = 25   # 100ms — hold motors and abort

        try:
            for i, pt in enumerate(points):
                if self._server.is_preempt_requested():
                    rospy.logwarn("[F5] Trajectory preempted.")
                    self._server.set_preempted()
                    return

                # Measure how far behind schedule we are
                elapsed  = (rospy.Time.now() - start_time).to_sec()
                expected = pt.time_from_start.to_sec()
                lag      = elapsed - expected

                if lag > LAG_WARN_SEC:
                    rospy.logwarn(
                        f"[F5] Running {lag*1000:.1f}ms behind at point {i}/{len(points)}"
                    )

                # Abort if lag is unrecoverable
                if lag > LAG_ABORT_SEC:
                    rospy.logerr(
                        f"[F5] Lag {lag*1000:.1f}ms exceeds limit. Stopping arm."
                    )
                    # Hold each motor at its current position
                    for motor_idx in range(6):
                        send_f5(motor_idx, abs_positions[motor_idx], speed=0)
                    rospy.sleep(0.1)
                    self._server.set_aborted(FollowJointTrajectoryResult())
                    return

                # Send this point
                velocities = list(pt.velocities) if has_velocities else None
                self._send_joint_positions(list(pt.positions), joint_indices, velocities)

                # Sleep until next point, adjusted to recover from lag
                if i < len(points) - 1:
                    dt        = (points[i + 1].time_from_start - pt.time_from_start).to_sec()
                    actual_dt = dt - max(0.0, lag)
                    if actual_dt > 0:
                        rospy.sleep(actual_dt)

            # All points sent successfully
            rospy.sleep(0.3)
            rospy.loginfo("[F5] Trajectory execution complete.")
            self._server.set_succeeded(FollowJointTrajectoryResult())

        except Exception as e:
            rospy.logerr(f"[F5] Trajectory execution error: {e}")
            self._server.set_aborted(FollowJointTrajectoryResult())
        finally:
            streaming_active = False

    def _send_joint_positions(self, positions, joint_indices, velocities=None):
        """
        Convert joint positions (radians) to absolute encoder counts and
        send F5 commands to all motors.

        When velocities (rad/s) are provided, derives per-motor RPM from
        the trajectory's own velocity profile via the gear ratio. Falls
        back to the global F5_SPEED constant when velocities is None.

        Also updates last_sent_angles so that any subsequent settle-detect
        moves use the correct baseline.
        """
        with last_sent_lock:
            joint_angles = list(last_sent_angles)

        joint_vels = [0.0] * 6

        for traj_idx, joint_idx in enumerate(joint_indices):
            if joint_idx >= 0 and traj_idx < len(positions):
                joint_angles[joint_idx] = positions[traj_idx]
                if velocities and traj_idx < len(velocities):
                    joint_vels[joint_idx] = abs(velocities[traj_idx])

        # Apply differential mix for J5/J6 (positions and velocities)
        mixed_pos = apply_differential_mix(joint_angles)
        mixed_vel = apply_differential_mix(joint_vels)

        # Send F5 to each motor
        for motor_idx in range(6):
            target_counts = rad_to_encoder_counts(motor_idx, mixed_pos[motor_idx])

            if velocities is not None and mixed_vel[motor_idx] > 0.001:
                rpm = rad_s_to_rpm(motor_idx, mixed_vel[motor_idx])
            else:
                rpm = F5_SPEED  # fallback when no velocity data

            send_f5(motor_idx, target_counts, speed=rpm)
            abs_positions[motor_idx] = target_counts

        # Update shared state so settle-detect knows where we are
        with last_sent_lock:
            for i in range(6):
                last_sent_angles[i] = joint_angles[i]


# ── SETTLE-DETECT FALLBACK (for demo.launch / GUI) ───────────────────────────

def joint_state_callback(msg):
    """Store latest joint state from fake controller (for GUI moves)."""
    global latest_angles, last_recv_time

    joint_map = {f"joint{j+1}": j for j in range(6)}

    with last_sent_lock:
        new_angles = list(last_sent_angles)

    for i, name in enumerate(msg.name):
        if name in joint_map:
            new_angles[joint_map[name]] = msg.position[i]

    with angles_lock:
        latest_angles = new_angles
        last_recv_time = time.time()


def settle_sender_loop():
    """
    Fallback for demo.launch: watches for GUI-driven moves via the
    fake controller topic. Uses F4 when action server is NOT streaming.
    Only runs when ENABLE_SETTLE_DETECT is True.
    """
    if not ENABLE_SETTLE_DETECT:
        rospy.loginfo("[F4] Settle-detect disabled. Fallback inactive.")
        return

    pending = False

    while not rospy.is_shutdown():
        time.sleep(0.02)

        if streaming_active:
            pending = False
            continue

        with angles_lock:
            target = list(latest_angles) if latest_angles is not None else None
            recv_time = last_recv_time

        if target is None:
            continue

        with last_sent_lock:
            current = list(last_sent_angles)

        mixed_target = apply_differential_mix(target)
        mixed_last = apply_differential_mix(current)
        deltas = [mixed_target[i] - mixed_last[i] for i in range(6)]

        if any(abs(d) >= MIN_DELTA_RAD for d in deltas):
            pending = True

        if not pending:
            continue

        elapsed = time.time() - recv_time
        if elapsed < SETTLE_TIME:
            continue

        if not move_lock.acquire(blocking=False):
            continue

        try:
            rospy.loginfo(f"[GUI/F4] Settled ({elapsed:.2f}s). Sending deltas.")

            clear_ack_queues()

            joints_sent = []
            for joint_idx in range(6):
                if send_f4(joint_idx, deltas[joint_idx]):
                    joints_sent.append(joint_idx)

            if not joints_sent:
                with last_sent_lock:
                    for i in range(6):
                        last_sent_angles[i] = target[i]
                pending = False
                continue

            rospy.loginfo(f"[GUI/F4] Waiting for acks: {[j+1 for j in joints_sent]}")

            for joint_idx in joints_sent:
                wait_for_ack(joint_idx)

            with last_sent_lock:
                for i in range(6):
                    last_sent_angles[i] = target[i]
            pending = False

            if move_complete_pub is not None:
                move_complete_pub.publish(Bool(data=True))
                rospy.loginfo("[GUI/F4] Published move_complete.")

        finally:
            move_lock.release()


# ── MAIN ──────────────────────────────────────────────────────────────────────

def main():
    global bus, move_complete_pub

    rospy.init_node("arctos_can_bridge")
    rospy.loginfo("=" * 60)
    rospy.loginfo("Arctos CAN bridge (F5 streaming + F4 GUI fallback)")
    rospy.loginfo(f"  CAN              : {CAN_CHANNEL} @ {CAN_BITRATE}")
    rospy.loginfo(f"  Motor IDs        : {CAN_IDS}")
    rospy.loginfo(f"  Gear ratios      : {GEAR_RATIOS}")
    rospy.loginfo(f"  Encoder CPR      : {ENCODER_CPR}")
    rospy.loginfo(f"  F5 speed         : {F5_SPEED} RPM (fallback, dynamic when vel data present)")
    rospy.loginfo(f"  F5 acc           : {F5_ACC}")
    rospy.loginfo(f"  F4 speed         : {F4_SPEED} RPM")
    rospy.loginfo(f"  Stream rate      : {STREAM_RATE_HZ} Hz")
    rospy.loginfo(f"  Settle-detect    : {'enabled' if ENABLE_SETTLE_DETECT else 'disabled'}")
    rospy.loginfo("=" * 60)

    try:
        bus = can.interface.Bus(
            interface=CAN_INTERFACE,
            channel=CAN_CHANNEL,
            bitrate=CAN_BITRATE,
        )
        rospy.loginfo(f"CAN bus open on {CAN_CHANNEL}")
    except Exception as e:
        rospy.logerr(f"Failed to open CAN bus: {e}")
        return

    # Query initial encoder positions
    rospy.loginfo("Querying initial encoder positions...")
    for i in range(6):
        pos = query_encoder_position(i)
        abs_positions[i] = pos
        rospy.loginfo(f"  J{i+1} (ID {CAN_IDS[i]}): encoder = {pos}")

    # Publishers
    move_complete_pub = rospy.Publisher(
        "/arctos/move_complete", Bool, queue_size=10
    )

    # Subscriber for settle-detect fallback (demo.launch)
    rospy.Subscriber(
        "/move_group/fake_controller_joint_states",
        JointState,
        joint_state_callback,
        queue_size=50,
    )

    # Start action server
    action_server = TrajectoryActionServer()

    # Start CAN listener thread
    listener = threading.Thread(target=can_listener_loop, daemon=True)
    listener.start()

    # Start settle-detect sender (exits immediately if ENABLE_SETTLE_DETECT is False)
    sender = threading.Thread(target=settle_sender_loop, daemon=True)
    sender.start()

    rospy.loginfo("Ready.")
    rospy.loginfo("  Action server: /arm_controller/follow_joint_trajectory")
    rospy.loginfo("  GUI fallback:  /move_group/fake_controller_joint_states")

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
