#!/usr/bin/env python3
"""
Arctos Arm - Cartesian Trajectory Runner
----------------------------------------
Add your waypoints to the WAYPOINTS list below, then run with:
    python3 arctos_trajectory.py

Make sure the following are already running before executing this script:
    Terminal 1: roslaunch arctos_config demo.launch
    Terminal 2: rosrun moveo_moveit interface.py
    Terminal 3: rosrun moveo_moveit transform.py  (or roscan.py via run.sh)
"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi


# ─────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────

# Speed scaling: 0.0 (slowest) to 1.0 (full speed)
# Start low (0.1) until you've verified the path is safe
VELOCITY_SCALING     = 0.1
ACCELERATION_SCALING = 0.1

# Cartesian path resolution in meters — smaller = smoother but slower to plan
CARTESIAN_STEP = 0.01

# If the planner can't achieve at least this fraction of the path, it will abort
# 1.0 = require 100% of the path to be reachable
MIN_FRACTION = 0.95

# ─────────────────────────────────────────────
#  WAYPOINTS
#  Each waypoint is (x, y, z, qx, qy, qz, qw)
#  Position in meters, orientation as quaternion
#
#  Tips:
#    - Keep orientation (qx, qy, qz, qw) = (0, 0, 0, 1) to start
#      (end effector pointing straight down from home)
#    - Run `rostopic echo /move_group/fake_controller_joint_states`
#      to read current pose while jogging in RViz
#    - The arm moves through ALL waypoints in order as one smooth path
# ─────────────────────────────────────────────

WAYPOINTS = [
    # (  x,     y,     z,    qx,   qy,   qz,   qw  )
    (  0.4,  -0.31,  0.1796,  0.0,  0.0,  0.0,  1.0 ),  # Point 1
    (  0.30,  -0.31,  0.1796,  0.0,  0.0,  0.0,  1.0 ),  # Point 2
    (  0.40,  -0.31,  0.1796,  0.0,  0.0,  0.0,  1.0 ),  # Point 3
    (  0.30,  -0.31,  0.1796,  0.0,  0.0,  0.0,  1.0 ),  # Point 4 — back to start Z
]

# Set to True to return to home position after completing the trajectory
RETURN_HOME = True


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────

def make_pose(x, y, z, qx, qy, qz, qw):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def run_trajectory():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("arctos_trajectory_runner", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
    group.set_max_acceleration_scaling_factor(ACCELERATION_SCALING)

    rospy.loginfo("=" * 50)
    rospy.loginfo("Arctos trajectory runner started")
    rospy.loginfo(f"Planning frame:  {group.get_planning_frame()}")
    rospy.loginfo(f"End effector:    {group.get_end_effector_link()}")
    rospy.loginfo(f"Waypoints:       {len(WAYPOINTS)}")
    rospy.loginfo("=" * 50)

    # Build waypoint list
    poses = [make_pose(*wp) for wp in WAYPOINTS]

    rospy.loginfo("Planning Cartesian path...")
    (plan, fraction) = group.compute_cartesian_path(
        poses,
        CARTESIAN_STEP,
        True  # jump threshold — 0.0 disables jump detection
    )

    rospy.loginfo(f"Path coverage: {fraction * 100:.1f}%")

    if fraction < MIN_FRACTION:
        rospy.logerr(
            f"Only {fraction * 100:.1f}% of the path is reachable "
            f"(minimum required: {MIN_FRACTION * 100:.1f}%). Aborting."
        )
        rospy.logerr("Check your waypoints are within the arm's workspace.")
        return

    # Retime the trajectory to respect velocity/acceleration limits
    plan = group.retime_trajectory(
        robot.get_current_state(),
        plan,
        VELOCITY_SCALING,
        ACCELERATION_SCALING
    )

    rospy.loginfo("Executing trajectory — watch the arm!")
    rospy.sleep(1.0)  # brief pause before moving

    success = group.execute(plan, wait=True)
    group.stop()
    group.clear_pose_targets()

    if success:
        rospy.loginfo("Trajectory completed successfully.")
    else:
        rospy.logerr("Trajectory execution failed.")
        return

    if RETURN_HOME:
        rospy.loginfo("Returning to home position...")
        group.set_named_target("home")
        group.go(wait=True)
        group.stop()
        rospy.loginfo("Home reached.")

    rospy.loginfo("Done.")


if __name__ == "__main__":
    try:
        run_trajectory()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nAborted by user.")
