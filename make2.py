#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import String  
from tf.transformations import quaternion_from_euler
import copy

# --- Setup Publisher for rosjog.py ---
ui_pub = rospy.Publisher('/ui_command', String, queue_size=50) # Increased queue size for streaming

def stream_plan_to_physical_arm(plan):
    """
    Extracts every waypoint from the MoveIt plan and streams them 
    to the physical arm, waiting the appropriate amount of time between each.
    """
    points = plan.joint_trajectory.points
    
    if not points:
        rospy.logwarn("Trajectory is empty! Nothing to execute.")
        return

    rospy.loginfo(f"Streaming {len(points)} waypoints to the physical arm...")

    for i in range(len(points)):
        point = points[i]
        joints = point.positions
        
        # Only send first 3 joints (x, y, z), zero out joints 4, 5, 6 (not plugged in)
        safe_joints = list(joints[:3]) + [0.0, 0.0, 0.0]

        # Format: go_to_joint_state,j1,j2,j3,j4,j5,j6
        msg = "go_to_joint_state," + ",".join([str(j) for j in safe_joints])
        ui_pub.publish(msg)
        
        # MoveIt calculates exact timing for smooth motion. We need to respect it.
        if i < len(points) - 1:
            current_time = point.time_from_start.to_sec()
            next_time = points[i+1].time_from_start.to_sec()
            time_to_wait = next_time - current_time
            
            if time_to_wait > 0:
                rospy.sleep(max(time_to_wait, 0.05))  # minimum 50ms between waypoints
                
    rospy.loginfo("Finished streaming path.")

def send_to_physical_arm(group):
    """Used for single-point jumps (like returning home)"""
    joints = group.get_current_joint_values()
    msg = "go_to_joint_state," + ",".join([str(j) for j in joints])
    ui_pub.publish(msg)
    rospy.loginfo(f"Sent home command to CAN: {msg}")

# --- Standard MoveIt Setup ---
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("cartesian_two_points")

# Give the publisher a moment to connect to rosjog-2.py
rospy.sleep(1)

group = moveit_commander.MoveGroupCommander("arm")
qx, qy, qz, qw = quaternion_from_euler(0, -1.5708, 0)
waypoints = []
start_pose = group.get_current_pose().pose

x, y, z = 0.3, -0.21, 0.1796
delta = 0.1

# Point 1
pose1 = copy.deepcopy(start_pose)
pose1.position.x = x + delta
pose1.position.y = y
pose1.position.z = z
pose1.orientation.x = qx
pose1.orientation.y = qy
pose1.orientation.z = qz
pose1.orientation.w = qw
waypoints.append(pose1)

# Point 2
pose2 = copy.deepcopy(start_pose)
pose2.position.x = x - delta
pose2.position.y = y
pose2.position.z = z
pose2.orientation.x = qx
pose2.orientation.y = qy
pose2.orientation.z = qz
pose2.orientation.w = qw
waypoints.append(pose2)

# Compute Cartesian Path
(plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, True)

if fraction > 0.9: 
    print("Path fraction:", fraction)
    
    # 1. Execute in RViz (Simulation)
    group.execute(plan, wait=True)
    
    # 2. Stream the generated trajectory to the physical arm (joints 4,5,6 zeroed)
    stream_plan_to_physical_arm(plan)
else:
    print("Path planning failed or fraction too low.")

rospy.sleep(1)

# Return Home
rospy.loginfo("Returning Home...")
group.set_named_target("home")
(home_success, home_plan, _, _) = group.plan()

if home_success:
    group.execute(home_plan, wait=True)
    # Stream the home path so it doesn't crash into things on the way back!
    #stream_plan_to_physical_arm(home_plan)
else:
    rospy.logwarn("Failed to plan path home.")

group.stop()
