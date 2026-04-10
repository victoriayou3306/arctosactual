#!/usr/bin/env python3
"""
roscanPD.py — Strict PDF Implementation of CSV Control (PD Only)
---------------------------------------------------------------
Implements the CSV algorithm from the provided documentation,
with the Integrator explicitly removed for pure PD tuning.
"""

import math
import threading
import queue
import time
import can
import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray
from control_msgs.msg import (
    FollowJointTrajectoryAction,
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

F5_SPEED = 200
F5_ACC   = 200

# Conservative velocity limits q_max
F5_RPM_MIN = 20
F5_RPM_MAX = 500

# --- PD CONTROLLER CONFIGURATION ---
Kp = [1, 0, 0.5, 0, 0, 0]        # Tuning Step 2 (Stiffness)
Kd = [0, 0, 0, 0, 0, 0]                 # Tuning Step 3 (Damping)

# Filter tau=10ms, Ts=5ms -> alpha = 10/(10+5)
ALPHA = 0.66

encoder_feedback_rad       = [0.0] * 6
encoder_feedback_vel_rad_s = [0.0] * 6
encoder_feedback_prev_rad  = [0.0] * 6
encoder_feedback_prev_time = [0.0] * 6

filtered_rpm_prev = [0.0] * 6

STREAM_RATE_HZ = 200
SETTLE_TIME    = 0.15
MIN_DELTA_RAD  = 0.005
F4_SPEED       = 200
F4_ACC         = 20
ACK_TIMEOUT    = 15.0

# ── STATE ─────────────────────────────────────────────────────────────────────

bus               = None
move_complete_pub = None
pd_state_pub      = None
abs_positions     = [0] * 6
JOINT_NAMES       = [f"joint{i+1}" for i in range(6)]
last_sent_angles  = [0.0] * 6
last_sent_lock    = threading.Lock()
latest_angles     = None
last_recv_time    = None
angles_lock       = threading.Lock()
move_lock         = threading.Lock()
ack_queues        = [queue.Queue() for _ in range(6)]
CAN_ID_TO_JOINT   = {cid: idx for idx, cid in enumerate(CAN_IDS)}
streaming_active  = False

# ── CAN HELPERS ───────────────────────────────────────────────────────────────

def calculate_crc(can_id, data_bytes):
    return (can_id + sum(data_bytes)) & 0xFF


def apply_differential_mix(values):
    mixed = list(values)
    mixed[4] = (values[4] + values[5]) / 2.0
    mixed[5] = (values[4] - values[5]) / 2.0
    return mixed


def rad_to_encoder_counts(joint_idx, angle_rad):
    ratio = GEAR_RATIOS[joint_idx]
    deg = math.degrees(angle_rad)
    if INVERT_DIRECTION[joint_idx]:
        deg = -deg
    return int(deg / 360.0 * ENCODER_CPR * ratio)


def send_f5(joint_idx, abs_counts, speed=None, acc=None):
    if speed is None: speed = F5_SPEED
    if acc is None:   acc   = F5_ACC
    can_id   = CAN_IDS[joint_idx]
    speed_hi = (speed >> 8) & 0xFF
    speed_lo = speed & 0xFF
    abs_24   = abs_counts & 0xFFFFFF
    axis_b0  = (abs_24 >> 16) & 0xFF
    axis_b1  = (abs_24 >> 8)  & 0xFF
    axis_b2  = abs_24 & 0xFF
    data = [0xF5, speed_hi, speed_lo, acc, axis_b0, axis_b1, axis_b2]
    data.append(calculate_crc(can_id, data))
    print(can_id, data)
    try:
        bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))
        return True
    except Exception as e:
        rospy.logerr(f"CAN F5 error: {e}")
        return False


def send_f4(joint_idx, delta_rad):
    if abs(delta_rad) < MIN_DELTA_RAD: return False
    can_id    = CAN_IDS[joint_idx]
    ratio     = GEAR_RATIOS[joint_idx]
    delta_deg = math.degrees(delta_rad)
    if INVERT_DIRECTION[joint_idx]: delta_deg = -delta_deg
    rel_axis = int(delta_deg / 360.0 * ENCODER_CPR * ratio)
    if rel_axis == 0: return False
    speed_hi = (F4_SPEED >> 8) & 0xFF
    speed_lo = F4_SPEED & 0xFF
    rel_24   = rel_axis & 0xFFFFFF
    axis_b0  = (rel_24 >> 16) & 0xFF
    axis_b1  = (rel_24 >> 8)  & 0xFF
    axis_b2  = rel_24 & 0xFF
    data = [0xF4, speed_hi, speed_lo, F4_ACC, axis_b0, axis_b1, axis_b2]
    data.append(calculate_crc(can_id, data))
    try:
        bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))
        return True
    except Exception:
        return False


def clear_ack_queues():
    for q in ack_queues:
        while not q.empty():
            try: q.get_nowait()
            except queue.Empty: break


def wait_for_ack(joint_idx):
    deadline = time.time() + ACK_TIMEOUT
    while time.time() < deadline:
        try:
            rem = deadline - time.time()
            if rem <= 0: break
            status = ack_queues[joint_idx].get(timeout=min(rem, 0.5))
            if status == 0x02:             return True
            elif status in (0x00, 0x03):   return False
        except queue.Empty:
            continue
    return False


def query_encoder_position(joint_idx):
    can_id = CAN_IDS[joint_idx]
    data = [0x31]
    data.append(calculate_crc(can_id, data))
    try:
        bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))
    except Exception:
        return 0
    deadline = time.time() + 0.5
    while time.time() < deadline:
        try:
            resp = bus.recv(timeout=0.1)
            if resp and resp.arbitration_id == can_id and len(resp.data) >= 8 and resp.data[0] == 0x31:
                raw = 0
                for i in range(1, 7): raw = (raw << 8) | resp.data[i]
                if raw >= (1 << 47): raw -= (1 << 48)
                return raw
        except Exception:
            continue
    return 0

# ── CAN LISTENER & ENCODER THREADS ───────────────────────────────────────────

def can_listener_loop():
    while not rospy.is_shutdown():
        try:
            msg = bus.recv(timeout=0.1)
            if msg is None: continue
            arb_id = msg.arbitration_id
            data   = msg.data
            if arb_id in CAN_ID_TO_JOINT and len(data) >= 2:
                joint_idx = CAN_ID_TO_JOINT[arb_id]
                if data[0] == 0xF4:
                    ack_queues[joint_idx].put(data[1])
                elif data[0] == 0x31 and len(data) >= 8:
                    raw = 0
                    for i in range(1, 7): raw = (raw << 8) | data[i]
                    if raw >= (1 << 47): raw -= (1 << 48)

                    angle_rad = math.radians(raw / (ENCODER_CPR * GEAR_RATIOS[joint_idx]) * 360.0)
                    if INVERT_DIRECTION[joint_idx]: angle_rad = -angle_rad

                    encoder_feedback_rad[joint_idx] = angle_rad
                    now = time.time()
                    dt  = now - encoder_feedback_prev_time[joint_idx]
                    if 0 < dt < 0.5:
                        encoder_feedback_vel_rad_s[joint_idx] = (
                            (angle_rad - encoder_feedback_prev_rad[joint_idx]) / dt
                        )

                    encoder_feedback_prev_rad[joint_idx]  = angle_rad
                    encoder_feedback_prev_time[joint_idx] = now
        except Exception:
            pass


def encoder_query_loop():
    ENCODER_QUERY_HZ = 120
    joint_idx = 0
    rate = rospy.Rate(ENCODER_QUERY_HZ)
    while not rospy.is_shutdown():
        if not streaming_active:
            rate.sleep()
            continue
        can_id = CAN_IDS[joint_idx]
        data = [0x31]
        data.append(calculate_crc(can_id, data))
        try:
            bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))
        except Exception:
            pass
        joint_idx = (joint_idx + 1) % 6
        rate.sleep()

# ── ACTION SERVER ─────────────────────────────────────────────────────────────

class TrajectoryActionServer:
    def __init__(self):
        self._server = actionlib.SimpleActionServer(
            "/arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._execute_cb, auto_start=False)
        self._server.start()

    def _execute_cb(self, goal):
        global streaming_active
        points = goal.trajectory.points
        if not points:
            self._server.set_succeeded(FollowJointTrajectoryResult())
            return

        joint_indices  = [JOINT_NAMES.index(n) if n in JOINT_NAMES else -1
                          for n in goal.trajectory.joint_names]
        has_velocities = (len(points[0].velocities) == len(points[0].positions)
                          and len(points[0].velocities) > 0)

        streaming_active = True
        rate       = rospy.Rate(STREAM_RATE_HZ)
        start_time = rospy.Time.now()
        point_idx  = 0

        try:
            while not rospy.is_shutdown():
                if self._server.is_preempt_requested():
                    self._server.set_preempted()
                    return

                elapsed = (rospy.Time.now() - start_time).to_sec()
                while (point_idx < len(points) - 1 and
                       points[point_idx + 1].time_from_start.to_sec() <= elapsed):
                    point_idx += 1

                pt = points[point_idx]
                if point_idx < len(points) - 1:
                    pt_next = points[point_idx + 1]
                    t0 = pt.time_from_start.to_sec()
                    t1 = pt_next.time_from_start.to_sec()
                    dt = t1 - t0
                    alpha = min(1.0, max(0.0, (elapsed - t0) / dt)) if dt > 0 else 1.0
                    interp_positions = [
                        pt.positions[i] + alpha * (pt_next.positions[i] - pt.positions[i])
                        for i in range(len(pt.positions))
                    ]
                    interp_velocities = [
                        pt.velocities[i] + alpha * (pt_next.velocities[i] - pt.velocities[i])
                        for i in range(len(pt.velocities))
                    ] if has_velocities else None
                else:
                    interp_positions  = list(pt.positions)
                    interp_velocities = list(pt.velocities) if has_velocities else None
                    if elapsed >= pt.time_from_start.to_sec():
                        self._send_joint_positions(interp_positions, joint_indices, interp_velocities)
                        break

                self._send_joint_positions(interp_positions, joint_indices, interp_velocities)
                rate.sleep()

            final_pt = points[-1]
            self._send_joint_positions(
                list(final_pt.positions), joint_indices,
                list(final_pt.velocities) if has_velocities else None
            )
            rospy.sleep(0.3)
            self._server.set_succeeded(FollowJointTrajectoryResult())
        finally:
            streaming_active = False

    def _send_joint_positions(self, positions, joint_indices, velocities=None):
        with last_sent_lock:
            joint_angles = list(last_sent_angles)
        joint_vels = [0.0] * 6

        for traj_idx, joint_idx in enumerate(joint_indices):
            if joint_idx >= 0 and traj_idx < len(positions):
                joint_angles[joint_idx] = positions[traj_idx]
                if velocities and traj_idx < len(velocities):
                    joint_vels[joint_idx] = velocities[traj_idx]

        mixed_target_pos = apply_differential_mix(joint_angles)
        mixed_target_vel = apply_differential_mix(joint_vels)
        mixed_curr_pos   = encoder_feedback_rad
        mixed_curr_vel   = encoder_feedback_vel_rad_s
        rpm_floor        = rospy.get_param('/arctos/rpm_floor', F5_RPM_MIN)

        computed_vels = [0.0] * 6

        for i in range(6):
            # Equation: e[k] = q_d[k] - q[k]
            pos_error = mixed_target_pos[i] - mixed_curr_pos[i]

            # Equation: q_msg_raw = Kp*e - Kd*q_dot + feedforward
            feedforward     = mixed_target_vel[i]
            p_term          = Kp[i] * pos_error
            d_term          = Kd[i] * mixed_curr_vel[i]
            total_vel_rad_s = feedforward + p_term - d_term
            computed_vels[i] = total_vel_rad_s

            raw_rpm = (abs(total_vel_rad_s) * GEAR_RATIOS[i] * 60.0) / (2.0 * math.pi)

            # Equation: q_msg_filtered = alpha*q_msg_filtered[k-1] + (1-alpha)*q_msg[k]
            filtered_rpm      = (ALPHA * filtered_rpm_prev[i]) + ((1.0 - ALPHA) * raw_rpm)
            filtered_rpm_prev[i] = filtered_rpm

            # Saturation: sat(q_msg_filtered, -q_max, +q_max)
            final_rpm = int(max(rpm_floor, min(filtered_rpm, F5_RPM_MAX)))

            target_counts = rad_to_encoder_counts(i, mixed_target_pos[i])
            send_f5(i, target_counts, speed=final_rpm)
            abs_positions[i] = target_counts

        # ── Publish PD diagnostics ──
        if pd_state_pub:
            errors_out = [mixed_target_pos[i] - mixed_curr_pos[i] for i in range(6)]
            qdots_out  = [computed_vels[i] for i in range(6)]
            pd_msg = Float32MultiArray()
            pd_msg.data = errors_out + qdots_out
            pd_state_pub.publish(pd_msg)

        with last_sent_lock:
            for i in range(6):
                last_sent_angles[i] = joint_angles[i]

# ── SETTLE FALLBACK ───────────────────────────────────────────────────────────

def joint_state_callback(msg):
    global latest_angles, last_recv_time
    joint_map = {f"joint{j+1}": j for j in range(6)}
    with last_sent_lock: new_angles = list(last_sent_angles)
    for i, name in enumerate(msg.name):
        if name in joint_map: new_angles[joint_map[name]] = msg.position[i]
    with angles_lock:
        latest_angles  = new_angles
        last_recv_time = time.time()


def settle_sender_loop():
    pending = False
    while not rospy.is_shutdown():
        time.sleep(0.02)
        if streaming_active:
            pending = False
            continue
        with angles_lock:
            target    = list(latest_angles) if latest_angles is not None else None
            recv_time = last_recv_time
        if target is None: continue
        with last_sent_lock: current = list(last_sent_angles)
        mixed_target = apply_differential_mix(target)
        mixed_last   = apply_differential_mix(current)
        deltas = [mixed_target[i] - mixed_last[i] for i in range(6)]
        if any(abs(d) >= MIN_DELTA_RAD for d in deltas): pending = True
        if not pending: continue
        if time.time() - recv_time < SETTLE_TIME: continue
        if not move_lock.acquire(blocking=False): continue
        try:
            clear_ack_queues()
            joints_sent = []
            for joint_idx in range(6):
                if send_f4(joint_idx, deltas[joint_idx]): joints_sent.append(joint_idx)
            if not joints_sent:
                with last_sent_lock:
                    for i in range(6): last_sent_angles[i] = target[i]
                pending = False
                continue
            for joint_idx in joints_sent: wait_for_ack(joint_idx)
            with last_sent_lock:
                for i in range(6): last_sent_angles[i] = target[i]
            pending = False
            if move_complete_pub: move_complete_pub.publish(Bool(data=True))
        finally:
            move_lock.release()

# ── MAIN ──────────────────────────────────────────────────────────────────────

def main():
    global bus, move_complete_pub, pd_state_pub
    rospy.init_node("arctos_can_bridge")
    rospy.loginfo("Arctos CAN bridge (PD Only CSV Control)")
    try:
        bus = can.interface.Bus(interface=CAN_INTERFACE, channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
    except Exception as e:
        rospy.logerr(f"Failed to open CAN bus: {e}")
        return

    for i in range(6):
        pos       = query_encoder_position(i)
        abs_positions[i] = pos
        angle_rad = math.radians(pos / (ENCODER_CPR * GEAR_RATIOS[i]) * 360.0)
        if INVERT_DIRECTION[i]: angle_rad = -angle_rad
        encoder_feedback_rad[i]       = angle_rad
        encoder_feedback_prev_rad[i]  = angle_rad
        encoder_feedback_prev_time[i] = time.time()

    move_complete_pub = rospy.Publisher("/arctos/move_complete", Bool,             queue_size=10)
    pd_state_pub      = rospy.Publisher("/arctos/pd_state",      Float32MultiArray, queue_size=10)
    rospy.Subscriber("/move_group/fake_controller_joint_states", JointState,
                     joint_state_callback, queue_size=50)

    action_server = TrajectoryActionServer()
    threading.Thread(target=can_listener_loop,  daemon=True).start()
    threading.Thread(target=encoder_query_loop, daemon=True).start()
    threading.Thread(target=settle_sender_loop, daemon=True).start()

    rospy.spin()
    bus.shutdown()


if __name__ == "__main__":
    try: main()
    except rospy.ROSInterruptException: pass
    except KeyboardInterrupt:
        if bus: bus.shutdown()
