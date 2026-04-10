#!/usr/bin/env python3
"""
arctos_pd_tune.py — Per-joint PD controller tuning for Arctos arm
------------------------------------------------------------------
Moves one joint at a time through step or oscillating (back-and-forth)
profiles so you can observe the PD controller response and tune Kp/Kd.

Uses the exact same interface as arctos_trajectory3.py:
  - MoveIt MoveGroupCommander("arm") for planning
  - group.set_joint_value_target() → group.plan() → retime_trajectory()
  - FollowJointTrajectory action server (served by roscanPD.py)

Also subscribes to /arctos/pd_state published by roscanPD.py to capture
live error and velocity-command data, then saves a CSV log you can plot.

Launch order:
  1. roslaunch arctos_config demo.launch
  2. python3 ~/arctosgui/roscanPD.py
  3. rosrun moveo_moveit interface.py
  4. rosrun moveo_moveit transform.py
  5. python3 arctos_pd_tune.py                    (this file)
"""

import sys
import os
import csv
import time
import threading
import rospy
import moveit_commander
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

# ─────────────────────────────────────────────────────────────────────────────
#  TUNING CONFIG  ← edit this section before each run
# ─────────────────────────────────────────────────────────────────────────────

TUNE_JOINT = 1

# "step" or "oscillate"
PROFILE = "oscillate"

AMPLITUDE_DEG = 30.0
REPEAT_COUNT  = 3
SEGMENT_DURATION     = 2.0
HOLD_TIME            = 1.5
VELOCITY_SCALING     = 0.3
ACCELERATION_SCALING = 0.6
RETURN_HOME          = True

RPM_FLOOR_RUN  = 200
RPM_FLOOR_HOME = 20

ACTION_TOPIC   = "/arm_controller/follow_joint_trajectory"
ACTION_TIMEOUT = 60.0

LOG_DIR = os.path.expanduser("~/arctosgui")

# ─────────────────────────────────────────────────────────────────────────────
#  PD STATE SUBSCRIBER  — logs continuously while _logging_active is True
# ─────────────────────────────────────────────────────────────────────────────

_pd_log         = []
_pd_lock        = threading.Lock()
_logging_active = False


def pd_state_callback(msg):
    if not _logging_active:
        return
    t    = rospy.Time.now().to_sec()
    data = list(msg.data)
    if len(data) < 12:
        return
    errors    = data[0:6]
    qdot_cmds = data[6:12]
    with _pd_lock:
        _pd_log.append((t, errors, qdot_cmds))


def start_pd_logging():
    global _logging_active
    with _pd_lock:
        _pd_log.clear()
    _logging_active = True


def stop_pd_logging():
    global _logging_active
    _logging_active = False


def flush_pd_log(joint_idx, filename):
    """Write accumulated PD log to a CSV file."""
    j = joint_idx  # 0-based
    os.makedirs(LOG_DIR, exist_ok=True)
    with _pd_lock:
        rows = list(_pd_log)

    if not rows:
        rospy.logwarn("PD log is empty — no data was captured.")
        return filename

    t0 = rows[0][0]  # use first sample as time zero

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_s",
            f"error_j{j+1}_rad",
            f"qdot_cmd_j{j+1}_rad_s",
            "error_j1", "error_j2", "error_j3", "error_j4", "error_j5", "error_j6",
            "qdot_j1",  "qdot_j2",  "qdot_j3",  "qdot_j4",  "qdot_j5",  "qdot_j6",
        ])
        for t, errors, qdots in rows:
            writer.writerow(
                [f"{t - t0:.4f}", f"{errors[j]:.6f}", f"{qdots[j]:.6f}"]
                + [f"{e:.6f}" for e in errors]
                + [f"{q:.6f}" for q in qdots]
            )

    rospy.loginfo(f"PD log saved: {filename}  ({len(rows)} samples)")
    return filename


# ─────────────────────────────────────────────────────────────────────────────
#  MOVEIT / ACTION SERVER HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def execute_via_action_server(client, plan):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = plan.joint_trajectory

    n_pts    = len(plan.joint_trajectory.points)
    duration = plan.joint_trajectory.points[-1].time_from_start.to_sec()
    rospy.loginfo(f"  Sending: {n_pts} pts, {duration:.2f}s")

    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(ACTION_TIMEOUT))

    if not finished:
        rospy.logerr("Action server timed out.")
        client.cancel_goal()
        return False

    state = client.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        return True
    else:
        rospy.logerr(f"Trajectory failed, action state={state}")
        return False


def plan_joint_target(group, robot, joint_targets_rad):
    group.set_joint_value_target(joint_targets_rad)
    result = group.plan()

    if isinstance(result, tuple):
        success = result[0]
        plan    = result[1]
    else:
        success = True
        plan    = result

    if not success or len(plan.joint_trajectory.points) == 0:
        rospy.logerr("Planning failed.")
        return None

    plan = group.retime_trajectory(
        robot.get_current_state(),
        plan,
        VELOCITY_SCALING,
        ACCELERATION_SCALING,
        algorithm="time_optimal_trajectory_generation",
    )
    return plan


# ─────────────────────────────────────────────────────────────────────────────
#  TUNING PROFILES
# ─────────────────────────────────────────────────────────────────────────────

def build_waypoints(home_joints, joint_idx, amplitude_rad, profile, repeat):
    waypoints = []
    base = list(home_joints)

    if profile == "step":
        for i in range(repeat):
            target = list(base)
            target[joint_idx] = base[joint_idx] + amplitude_rad
            waypoints.append((target, f"step_{i+1}_fwd"))
            waypoints.append((list(base), f"step_{i+1}_ret"))

    elif profile == "oscillate":
        signs = [+1, -1]
        for i in range(repeat):
            for s, label in zip(signs, ["pos", "neg"]):
                target = list(base)
                target[joint_idx] = base[joint_idx] + s * amplitude_rad
                waypoints.append((target, f"cycle_{i+1}_{label}"))
        waypoints.append((list(base), "return_home"))

    else:
        raise ValueError(f"Unknown profile: {profile!r}. Use 'step' or 'oscillate'.")

    return waypoints


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────

def run_tuning():
    import math

    joint_idx     = TUNE_JOINT - 1
    amplitude_rad = math.radians(AMPLITUDE_DEG)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("arctos_pd_tune", anonymous=True)

    rospy.Subscriber("/arctos/pd_state", Float32MultiArray, pd_state_callback, queue_size=500)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm")
    group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
    group.set_max_acceleration_scaling_factor(ACCELERATION_SCALING)

    rospy.loginfo(f"Connecting to action server: {ACTION_TOPIC}")
    client = actionlib.SimpleActionClient(ACTION_TOPIC, FollowJointTrajectoryAction)
    connected = client.wait_for_server(rospy.Duration(10.0))
    if not connected:
        rospy.logerr("Cannot connect to FollowJointTrajectory action server. Is roscanPD.py running?")
        return

    rospy.loginfo("Connected.")
    rospy.loginfo("=" * 60)
    rospy.loginfo("Arctos PD tuning run")
    rospy.loginfo(f"  Joint          : J{TUNE_JOINT}  (index {joint_idx})")
    rospy.loginfo(f"  Profile        : {PROFILE}")
    rospy.loginfo(f"  Amplitude      : ±{AMPLITUDE_DEG:.1f}°  ({amplitude_rad:.4f} rad)")
    rospy.loginfo(f"  Repeat count   : {REPEAT_COUNT}")
    rospy.loginfo(f"  Hold time      : {HOLD_TIME:.1f}s")
    rospy.loginfo(f"  Planning frame : {group.get_planning_frame()}")
    rospy.loginfo(f"  End effector   : {group.get_end_effector_link()}")
    rospy.loginfo("=" * 60)

    # ── Move to home first ──
    rospy.loginfo("Moving to home position...")
    rospy.set_param('/arctos/rpm_floor', RPM_FLOOR_RUN)

    group.set_named_target("home")
    home_result = group.plan()
    if isinstance(home_result, tuple):
        home_success, home_plan = home_result[0], home_result[1]
    else:
        home_success, home_plan = True, home_result

    if not home_success or len(home_plan.joint_trajectory.points) == 0:
        rospy.logerr("Could not plan to home. Aborting.")
        return

    home_plan = group.retime_trajectory(
        robot.get_current_state(),
        home_plan,
        VELOCITY_SCALING,
        ACCELERATION_SCALING,
        algorithm="time_optimal_trajectory_generation",
    )

    if not execute_via_action_server(client, home_plan):
        rospy.logerr("Failed to reach home. Aborting.")
        return

    rospy.sleep(HOLD_TIME)

    home_joints = list(group.get_current_joint_values())
    rospy.loginfo(f"Home joints: {[f'{v:.4f}' for v in home_joints]}")

    # ── Build waypoints ──
    waypoints = build_waypoints(home_joints, joint_idx, amplitude_rad, PROFILE, REPEAT_COUNT)
    rospy.loginfo(f"Waypoints to execute: {len(waypoints)}")
    for wp, label in waypoints:
        rospy.loginfo(f"  [{label}]  J{TUNE_JOINT} = {math.degrees(wp[joint_idx]):.2f}°")

    # ── Log file setup ──
    timestamp    = time.strftime("%Y%m%d_%H%M%S")
    log_filename = os.path.join(LOG_DIR, f"pd_tune_J{TUNE_JOINT}_{PROFILE}_{timestamp}.csv")

    # ── Start logging — runs for the entire tuning sequence ──
    start_pd_logging()

    # ── Execute each waypoint ──
    all_ok = True
    for wp_joints, label in waypoints:
        rospy.loginfo(f"--- {label} ---")

        plan = plan_joint_target(group, robot, wp_joints)
        if plan is None:
            rospy.logerr(f"Planning failed for waypoint [{label}]. Skipping.")
            all_ok = False
            continue

        ok = execute_via_action_server(client, plan)
        if not ok:
            rospy.logerr(f"Execution failed at [{label}].")
            all_ok = False

        rospy.loginfo(f"  Holding {HOLD_TIME:.1f}s...")
        rospy.sleep(HOLD_TIME)

    # ── Stop logging and save ──
    stop_pd_logging()
    flush_pd_log(joint_idx, log_filename)

    # ── Return home ──
    if RETURN_HOME:
        rospy.loginfo("Returning to home...")
        group.set_named_target("home")
        ret_result = group.plan()
        if isinstance(ret_result, tuple):
            ret_success, ret_plan = ret_result[0], ret_result[1]
        else:
            ret_success, ret_plan = True, ret_result

        if ret_success and len(ret_plan.joint_trajectory.points) > 0:
            ret_plan = group.retime_trajectory(
                robot.get_current_state(),
                ret_plan,
                VELOCITY_SCALING * 2,
                ACCELERATION_SCALING,
                algorithm="time_optimal_trajectory_generation",
            )
            rospy.set_param('/arctos/rpm_floor', RPM_FLOOR_RUN)
            execute_via_action_server(client, ret_plan)
            rospy.set_param('/arctos/rpm_floor', RPM_FLOOR_HOME)
        else:
            rospy.logwarn("Could not plan return to home.")

    rospy.loginfo("=" * 60)
    rospy.loginfo(f"Tuning run complete. {'All moves OK.' if all_ok else 'Some moves failed — check logs.'}")
    rospy.loginfo(f"Log file: {log_filename}")
    rospy.loginfo("Plot with:  python3 plot_pd_tune.py " + log_filename)
    rospy.loginfo("=" * 60)

    current_pose = group.get_current_pose().pose
    rospy.loginfo(f"Final end-effector pose: {current_pose}")


if __name__ == "__main__":
    try:
        run_tuning()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nAborted by user.")
