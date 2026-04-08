#!/usr/bin/env python3
"""
arctos_trajectory.py — Smooth trajectory runner for Arctos arm
--------------------------------------------------------------
Plans a continuous Cartesian path through all waypoints using
MoveIt's compute_cartesian_path, then sends the full trajectory
to roscan2.py's FollowJointTrajectory action server for smooth
F5-streamed execution.

No more stop-and-go between waypoints. The arm moves through all
points as one continuous motion.

Launch order:
  1. roslaunch arctos_config demo.launch
  2. python3 ~/arctosgui/roscan2.py
  3. rosrun moveo_moveit interface.py
  4. rosrun moveo_moveit transform.py
  5. python3 arctos_trajectory.py
"""

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


# ─────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────

VELOCITY_SCALING     = 0.6
ACCELERATION_SCALING = 1.0

# Cartesian path resolution in meters (smaller = smoother)
CARTESIAN_STEP = 0.005

# Abort if planner can't achieve this fraction of the path
MIN_FRACTION = 0.95

# Each target point: (x, y, z, qx, qy, qz, qw)
WAYPOINTS = [
    (0.20, -0.31, 0.0896, 0.0, 0.0, 0.0, 1.0),
    (0.0, -0.31, 0.0896, 0.0, 0.0, 0.0, 1.0),
    (-0.20, -0.31, 0.0896, 0.0, 0.0, 0.0, 1.0),
    (-0.20, -0.41, 0.0896, 0.0, 0.0, 0.0, 1.0),
    (0.0, -0.41, 0.0896, 0.0, 0.0, 0.0, 1.0),
    (0.2, -0.41, 0.0896, 0.0, 0.0, 0.0, 1.0),
]

RETURN_HOME = True

# Action server topic (must match roscan2.py)
ACTION_TOPIC = "/arm_controller/follow_joint_trajectory"

# Timeout for the action server to complete the trajectory
ACTION_TIMEOUT = 120.0  # seconds


# ─────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────

def make_pose(x, y, z, qx, qy, qz, qw):
    p = geometry_msgs.msg.Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


def execute_via_action_server(client, robot, group, plan):
    """
    Send a MoveIt plan to the FollowJointTrajectory action server.
    Returns True on success.
    """
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = plan.joint_trajectory

    rospy.loginfo(
        f"Sending trajectory: {len(plan.joint_trajectory.points)} points, "
        f"duration {plan.joint_trajectory.points[-1].time_from_start.to_sec():.2f}s"
    )

    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(ACTION_TIMEOUT))

    if not finished:
        rospy.logerr("Action server timed out.")
        client.cancel_goal()
        return False

    result = client.get_result()
    state = client.get_state()

    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Trajectory executed successfully.")
        return True
    else:
        rospy.logerr(f"Trajectory failed with state {state}")
        return False


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────

def run_trajectory():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("arctos_trajectory_runner", anonymous=True)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm")
    group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
    group.set_max_acceleration_scaling_factor(ACCELERATION_SCALING)

    # Connect to the action server
    rospy.loginfo(f"Connecting to action server: {ACTION_TOPIC}")
    client = actionlib.SimpleActionClient(ACTION_TOPIC, FollowJointTrajectoryAction)
    connected = client.wait_for_server(rospy.Duration(10.0))

    if not connected:
        rospy.logerr(
            "Could not connect to FollowJointTrajectory action server. "
            "Is roscan2.py running with the F5 streaming version?"
        )
        return

    rospy.loginfo("Connected to action server.")

    rospy.loginfo("=" * 50)
    rospy.loginfo("Arctos smooth trajectory runner")
    rospy.loginfo(f"  Planning frame : {group.get_planning_frame()}")
    rospy.loginfo(f"  End effector   : {group.get_end_effector_link()}")
    rospy.loginfo(f"  Waypoints      : {len(WAYPOINTS)}")
    rospy.loginfo(f"  Return home    : {RETURN_HOME}")
    rospy.loginfo("=" * 50)

    # ── Plan continuous Cartesian path through all waypoints ──
    poses = [make_pose(*wp) for wp in WAYPOINTS]

    rospy.loginfo("Planning Cartesian path through all waypoints...")
    (plan, fraction) = group.compute_cartesian_path(
        poses,
        CARTESIAN_STEP,
        True,  # jump threshold (0.0 = disabled)
    )

    rospy.loginfo(f"Path coverage: {fraction * 100:.1f}%")

    if fraction < MIN_FRACTION:
        rospy.logerr(
            f"Only {fraction * 100:.1f}% reachable "
            f"(need {MIN_FRACTION * 100:.1f}%). Aborting."
        )
        return

    # Retime to respect velocity/acceleration limits
    plan = group.retime_trajectory(
        robot.get_current_state(),
        plan,
        VELOCITY_SCALING,
        ACCELERATION_SCALING, algorithm="time_optimal_trajectory_generation"
    )

    # ── Execute via action server (smooth F5 streaming) ──
    rospy.loginfo("Executing trajectory via F5 streaming...")
    success = execute_via_action_server(client, robot, group, plan)

    if not success:
        rospy.logerr("Trajectory execution failed.")
        return

# ── Return to home ──
    if RETURN_HOME:
        rospy.loginfo("Planning return to home...")
        group.set_named_target("home")

        home_plan = group.plan()

        if isinstance(home_plan, tuple):
            plan_success = home_plan[0]
            home_plan = home_plan[1]
        else:
            plan_success = True

        if not plan_success or len(home_plan.joint_trajectory.points) == 0:
            rospy.logerr("Home planning failed.")
            return

        home_plan = group.retime_trajectory(
            robot.get_current_state(),
            home_plan,
            VELOCITY_SCALING,
            ACCELERATION_SCALING,
            algorithm="time_optimal_trajectory_generation",
        )

        rospy.loginfo(
            f"Home trajectory: {len(home_plan.joint_trajectory.points)} points, "
            f"duration {home_plan.joint_trajectory.points[-1].time_from_start.to_sec():.2f}s"
        )
        execute_via_action_server(client, robot, group, home_plan)

    rospy.loginfo("Done.")

if __name__ == "__main__":
    try:
        run_trajectory()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nAborted by user.")
