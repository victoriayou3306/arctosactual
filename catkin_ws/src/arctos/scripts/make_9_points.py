#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import copy
import time

qx, qy, qz, qw = quaternion_from_euler(0, -1.5708, 0)

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("cartesian_three_points")


group = moveit_commander.MoveGroupCommander("arm")
#rospy.sleep(2)

waypoints = []

# Starting pose
#start_pose = [0,0,0,0,0,0]
#group.go(start_pose, wait=True)

start_pose = group.get_current_pose().pose
#print(start_pose)
#waypoints.append(start_pose)


x = 0.1
y = -0.1
z = 0.35
delta = 0.1


# Point 1
pose1 = copy.deepcopy(start_pose)
pose1.position.x = x + delta
pose1.position.y = y - delta
pose1.position.z = z
pose1.orientation.x = qx
pose1.orientation.y = qy
pose1.orientation.z = qz
pose1.orientation.w = qw

waypoints.append(pose1)

# Point 2
pose2 = copy.deepcopy(start_pose)
pose2.position.x = x
pose2.position.y = y - delta
pose2.position.z = z
pose2.orientation.x = qx
pose2.orientation.y = qy
pose2.orientation.z = qz
pose2.orientation.w = qw
waypoints.append(pose2)

# Point 3
pose3 = copy.deepcopy(start_pose) 
pose3.position.x = x - delta
pose3.position.y = y - delta
pose3.position.z = z
pose3.orientation.x = qx
pose3.orientation.y = qy
pose3.orientation.z = qz
pose3.orientation.w = qw

waypoints.append(pose3)

# Point 4
pose4 = copy.deepcopy(start_pose)
pose4.position.x = x -delta
pose4.position.y = y
pose4.position.z = z
pose4.orientation.x = qx
pose4.orientation.y = qy
pose4.orientation.z = qz
pose4.orientation.w = qw

waypoints.append(pose4)

# Point 5
pose5 = copy.deepcopy(start_pose)
pose5.position.x = x
pose5.position.y = y
pose5.position.z = z
pose5.orientation.x = qx
pose5.orientation.y = qy
pose5.orientation.z = qz
pose5.orientation.w = qw

waypoints.append(pose5)

# Point 6
pose6 = copy.deepcopy(start_pose)
pose6.position.x = x + delta
pose6.position.y = y
pose6.position.z = z
pose6.orientation.x = qx
pose6.orientation.y = qy
pose6.orientation.z = qz
pose6.orientation.w = qw

waypoints.append(pose6)

# Point 7
pose7 =  copy.deepcopy(start_pose)
pose7.position.x = x + delta
pose7.position.y = y + delta
pose7.position.z = z
pose7.orientation.x = qx
pose7.orientation.y = qy
pose7.orientation.z = qz
pose7.orientation.w = qw
waypoints.append(pose7)

# Point 8
pose8 = copy.deepcopy(start_pose)
pose8.position.x = x
pose8.position.y = y + delta
pose8.position.z = z
pose8.orientation.x = qx
pose8.orientation.y = qy
pose8.orientation.z = qz
pose8.orientation.w = qw

waypoints.append(pose8)

# Point 9
pose9 = copy.deepcopy(start_pose)
pose9.position.x = x -delta
pose9.position.y = y + delta
pose9.position.z = z
pose9.orientation.x = qx
pose9.orientation.y = qy
pose9.orientation.z = qz
pose9.orientation.w = qw
waypoints.append(pose9)

(plan, fraction) = group.compute_cartesian_path(
    waypoints,
    0.01,   # eef_step (resolution)
    True     # jump_threshold
)
rospy.sleep(0.5)
#traj = plan.joint_trajectory
#for i in range(len(traj.points)):
  # joints = traj.points[i].positions
   # print("Waypoints", i, ":", joints)

print("Path fraction:", fraction)
group.execute(plan, wait=True)
group.stop()

rospy.sleep(1)

group.set_start_state_to_current_state()
time.sleep(10)
group.set_named_target("home")
group.go(wait=True)
group.stop()
