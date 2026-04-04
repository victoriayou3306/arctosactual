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
        
        # Format: go_to_joint_state,j1,j2,j3,j4,j5,j6
        msg = "go_to_joint_state," + ",".join([str(j) for j in joints])
        ui_pub.publish(msg)
        
        # MoveIt calculates exact timing for smooth motion. We need to respect it.
        if i < len(points) - 1:
            current_time = point.time_from_start.to_sec()
            next_time = points[i+1].time_from_start.to_sec()
            time_to_wait = next_time - current_time
            
            if time_to_wait > 0:
                rospy.sleep(time_to_wait)
                
    rospy.loginfo("Finished streaming path.")

def send_to_physical_arm(group):
    """Used for single-point jumps (like returning home)"""
    joints = group.get_current_joint_values()
    msg = "go_to_joint_state," + ",".join([str(j) for j in joints])
    ui_pub.publish(msg)
    rospy.loginfo(f"Sent home command to CAN: {msg}")

# --- Standard MoveIt Setup ---
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("cartesian_nine_points")

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
pose1.position.y = y - delta
pose1.position.z = z
pose1.orientation.x = qx; pose1.orientation.y = qy; pose1.orientation.z = qz; pose1.orientation.w = qw
waypoints.append(pose1)

# Point 2
pose2 = copy.deepcopy(start_pose)
pose2.position.x = x
pose2.position.y = y - delta
pose2.position.z = z
pose2.orientation.x = qx; pose2.orientation.y = qy; pose2.orientation.z = qz; pose2.orientation.w = qw
waypoints.append(pose2)

# Point 3
pose3 = copy.deepcopy(start_pose) 
pose3.position.x = x - delta
pose3.position.y = y - delta
pose3.position.z = z
pose3.orientation.x = qx; pose3.orientation.y = qy; pose3.orientation.z = qz; pose3.orientation.w = qw
waypoints.append(pose3)

# Point 4
pose4 = copy.deepcopy(start_pose)
pose4.position.x = x - delta
pose4.position.y = y
pose4.position.z = z
pose4.orientation.x = qx; pose4.orientation.y = qy; pose4.orientation.z = qz; pose4.orientation.w = qw
waypoints.append(pose4)

# Point 5
pose5 = copy.deepcopy(start_pose)
pose5.position.x = x
pose5.position.y = y
pose5.position.z = z
pose5.orientation.x = qx; pose5.orientation.y = qy; pose5.orientation.z = qz; pose5.orientation.w = qw
waypoints.append(pose5)

# Point 6
pose6 = copy.deepcopy(start_pose)
pose6.position.x = x + delta
pose6.position.y = y
pose6.position.z = z
pose6.orientation.x = qx; pose6.orientation.y = qy; pose6.orientation.z = qz; pose6.orientation.w = qw
waypoints.append(pose6)

# Point 7
pose7 =  copy.deepcopy(start_pose)
pose7.position.x = x + delta
pose7.position.y = y + delta
pose7.position.z = z
pose7.orientation.x = qx; pose7.orientation.y = qy; pose7.orientation.z = qz; pose7.orientation.w = qw
waypoints.append(pose7)

# Point 8
pose8 = copy.deepcopy(start_pose)
pose8.position.x = x
pose8.position.y = y + delta
pose8.position.z = z
pose8.orientation.x = qx; pose8.orientation.y = qy; pose8.orientation.z = qz; pose8.orientation.w = qw
waypoints.append(pose8)

# Point 9
pose9 = copy.deepcopy(start_pose)
pose9.position.x = x - delta
pose9.position.y = y + delta
pose9.position.z = z
pose9.orientation.x = qx; pose9.orientation.y = qy; pose9.orientation.z = qz; pose9.orientation.w = qw
waypoints.append(pose9)

# Compute Cartesian Path
(plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, True)

if fraction > 0.9: 
    print("Path fraction:", fraction)
    
    # 1. Execute in RViz (Simulation)
    group.execute(plan, wait=True)
    
    # 2. NEW: Stream the generated trajectory to the physical arm
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
    stream_plan_to_physical_arm(home_plan)
else:
    rospy.logwarn("Failed to plan path home.")

group.stop()
