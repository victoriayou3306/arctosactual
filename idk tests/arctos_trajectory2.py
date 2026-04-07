#!/usr/bin/env python3
"""
Arctos Arm - Cartesian Trajectory Runner
----------------------------------------
Add your waypoints to the WAYPOINTS list below, then run with:
    python3 arctos_trajectory.py

Make sure the following are already running before executing this script:
    Terminal 1: roslaunch arctos_config demo.launch
    Terminal 2: python3 ~/arctosgui/roscanvel.py
    Terminal 3: rosrun moveo_moveit interface.py
    Terminal 4: rosrun moveo_moveit transform.py
"""

import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg


# ─────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────

VELOCITY_SCALING     = 0.1
ACCELERATION_SCALING = 0.1

# Pause between waypoints. Must be long enough for roscan to:
#   1. Detect settle (0.15s)
#   2. Send F4 commands
#   3. Motors physically complete the move
# Increase if motors haven't finished before the next waypoint fires.
WAYPOINT_PAUSE = 5  # seconds

# Home position (used as the "previous" point for the first waypoint's
# distance calculation)
HOME = (0, 0.35574, 0.56674, 0.0, 0.0, 0.0, 1.0)

# Desired end-effector Cartesian speed in m/s.
# move_time for each segment = distance / E_SPEED
# This is approximate since joint-space motion won't follow a straight
# Cartesian line exactly, but it gives proportional timing.
E_SPEED = 0.2  # m/s

# ─────────────────────────────────────────────
#  WAYPOINTS
#  Each waypoint is (x, y, z, qx, qy, qz, qw)
# ─────────────────────────────────────────────

WAYPOINTS = [
    (  0.40, -0.31, 0.2032,  0.0, 0.0, 0.0, 1.0 ),  # Point 1
    (  0.30, -0.31, 0.2032,  0.0, 0.0, 0.0, 1.0 ),  # Point 2
    (  0.20, -0.31, 0.2032,  0.0, 0.0, 0.0, 1.0 ),  # Point 3
    (  0.20, -0.21, 0.2032,  0.0, 0.0, 0.0, 1.0 ),  # Point 4
]

RETURN_HOME = True


# ─────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────

def xyz_dist(a, b):
    """Euclidean distance between two waypoints (xyz only)."""
    return math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2 + (b[2]-a[2])**2)


def compute_move_times(waypoints, home, speed):
    """
    Compute per-segment move times based on Cartesian distance and
    desired end-effector speed. Returns a list the same length as waypoints.
    """
    times = []
    prev = home
    for wp in waypoints:
        dist = xyz_dist(prev, wp)
        t = dist / speed
        # Floor at a reasonable minimum so roscan doesn't get a near-zero time
        t = max(t, 0.1)
        times.append(t)
        prev = wp
    return times


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


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────

def run_trajectory():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("arctos_trajectory_runner", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm")

    group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
    group.set_max_acceleration_scaling_factor(ACCELERATION_SCALING)

    move_times = compute_move_times(WAYPOINTS, HOME, E_SPEED)

    rospy.loginfo("=" * 50)
    rospy.loginfo("Arctos trajectory runner started")
    rospy.loginfo(f"Planning frame:  {group.get_planning_frame()}")
    rospy.loginfo(f"End effector:    {group.get_end_effector_link()}")
    rospy.loginfo(f"Waypoints:       {len(WAYPOINTS)}")
    rospy.loginfo(f"E-E speed:       {E_SPEED} m/s")
    rospy.loginfo(f"Move times:      {[f'{t:.4f}s' for t in move_times]}")
    rospy.loginfo(f"Pause between:   {WAYPOINT_PAUSE}s")
    rospy.loginfo("=" * 50)

    for i, (wp, move_time) in enumerate(zip(WAYPOINTS, move_times)):
        # Set move_time BEFORE group.go() so roscan can latch it
        # as soon as it detects new joint states arriving
        rospy.set_param("/arctos/move_time", move_time)
        rospy.loginfo(
            f"Waypoint {i+1}/{len(WAYPOINTS)}: "
            f"pos={wp[:3]}  move_time={move_time:.4f}s"
        )

        target = make_pose(*wp)
        group.set_pose_target(target)
        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()

        if not success:
            rospy.logerr(f"Failed to reach waypoint {i+1}. Aborting.")
            return

        rospy.loginfo(f"Waypoint {i+1} published. Pausing {WAYPOINT_PAUSE}s for motors...")
        rospy.sleep(WAYPOINT_PAUSE)

    rospy.loginfo("All waypoints completed.")

    if RETURN_HOME:
        rospy.loginfo("Returning to home position...")
        # Give roscan a generous move_time for the home return
        home_dist = xyz_dist(WAYPOINTS[-1], HOME)
        home_time = max(0.1, home_dist / E_SPEED)
        rospy.set_param("/arctos/move_time", home_time)
        rospy.loginfo(f"Home move_time={home_time:.4f}s")

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
