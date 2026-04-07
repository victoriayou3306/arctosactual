#!/usr/bin/env python3
"""
arctos_trajectory.py — Waypoint trajectory runner for Arctos arm
----------------------------------------------------------------
Executes a list of Cartesian waypoints one at a time, waiting for
roscan2.py to confirm each physical move is complete before sending
the next one.

Requires roscan2.py to publish on /arctos/move_complete after each
settle-and-send cycle. If that topic isn't available (e.g. you haven't
patched roscan2.py yet), it falls back to a fixed delay.

Launch order:
  1. roslaunch arctos_config demo.launch
  2. python3 ~/arctosgui/roscan2.py
  3. rosrun moveo_moveit interface.py
  4. rosrun moveo_moveit transform.py
  5. python3 arctos_trajectory.py
"""

import sys
import threading
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Bool


# ─────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────

VELOCITY_SCALING     = 0.1
ACCELERATION_SCALING = 0.1

# Each waypoint: (x, y, z, qx, qy, qz, qw)
# Position in meters, orientation as quaternion.
# (0, 0, 0, 1) = end effector pointing straight down from home.
WAYPOINTS = [
    (0.40, -0.31, 0.1796, 0.0, 0.0, 0.0, 1.0),
    (0.30, -0.31, 0.1796, 0.0, 0.0, 0.0, 1.0),
    (0.40, -0.31, 0.1796, 0.0, 0.0, 0.0, 1.0),
    (0.30, -0.31, 0.1796, 0.0, 0.0, 0.0, 1.0),
]

RETURN_HOME = True

# If roscan2.py doesn't publish /arctos/move_complete, fall back to
# waiting this many seconds after each group.go() call.
FALLBACK_WAIT = 5.0

# Extra settling time (seconds) after receiving move_complete, to let
# motors fully stop before the next move is planned.
POST_MOVE_SETTLE = 0.5


# ─────────────────────────────────────────────
#  MOVE-COMPLETE SYNCHRONIZATION
# ─────────────────────────────────────────────

class MoveSync:
    """
    Listens for /arctos/move_complete from roscan2.py.

    IMPORTANT: call prepare() BEFORE sending a MoveIt command, then
    call wait_for_move() AFTER. This ensures the event is cleared
    before roscan2.py has a chance to publish, avoiding the race
    condition where the signal arrives before we start listening.
    """

    def __init__(self):
        self._event = threading.Event()
        self._connected = False

        # Check if the topic exists (i.e. roscan2 has been patched)
        topics = dict(rospy.get_published_topics())
        if "/arctos/move_complete" in topics:
            self._connected = True
            rospy.loginfo("Found /arctos/move_complete topic. Using CAN sync.")
        else:
            rospy.logwarn(
                "/arctos/move_complete not found. "
                f"Falling back to {FALLBACK_WAIT}s fixed delay."
            )

        rospy.Subscriber(
            "/arctos/move_complete", Bool, self._callback, queue_size=10
        )

    def _callback(self, msg):
        if msg.data:
            self._connected = True
            self._event.set()

    def prepare(self):
        """Call this BEFORE group.go() to arm the listener."""
        self._event.clear()

    def wait_for_move(self):
        """Call this AFTER group.go() returns. Blocks until confirmed."""
        if self._connected:
            got_it = self._event.wait(timeout=30.0)
            if got_it:
                rospy.loginfo("CAN bridge confirmed move complete.")
            else:
                rospy.logwarn("Timed out waiting for CAN bridge (30s). Continuing.")
        else:
            rospy.loginfo(f"Fixed delay: waiting {FALLBACK_WAIT}s...")
            rospy.sleep(FALLBACK_WAIT)

        rospy.sleep(POST_MOVE_SETTLE)


# ─────────────────────────────────────────────
#  TRAJECTORY EXECUTION
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


def run_trajectory():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("arctos_trajectory_runner", anonymous=True)

    group = moveit_commander.MoveGroupCommander("arm")
    group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
    group.set_max_acceleration_scaling_factor(ACCELERATION_SCALING)

    sync = MoveSync()

    rospy.loginfo("=" * 50)
    rospy.loginfo("Arctos trajectory runner")
    rospy.loginfo(f"  Planning frame : {group.get_planning_frame()}")
    rospy.loginfo(f"  End effector   : {group.get_end_effector_link()}")
    rospy.loginfo(f"  Waypoints      : {len(WAYPOINTS)}")
    rospy.loginfo(f"  Return home    : {RETURN_HOME}")
    rospy.loginfo("=" * 50)

    # ── Execute each waypoint individually ──
    for i, wp in enumerate(WAYPOINTS):
        label = f"[{i+1}/{len(WAYPOINTS)}]"
        rospy.loginfo(f"{label} Target: x={wp[0]:.3f} y={wp[1]:.3f} z={wp[2]:.3f}")

        target = make_pose(*wp)
        group.set_pose_target(target)

        # Clear the event BEFORE sending so we don't miss a fast response
        sync.prepare()

        success = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        if not success:
            rospy.logerr(f"{label} MoveIt planning/execution failed. Aborting.")
            return

        rospy.loginfo(f"{label} MoveIt trajectory published. Waiting for physical move...")
        sync.wait_for_move()
        rospy.loginfo(f"{label} Complete.")

    rospy.loginfo("All waypoints done.")

    # ── Return to home ──
    if RETURN_HOME:
        rospy.loginfo("Returning to home position...")
        group.set_named_target("home")

        sync.prepare()

        success = group.go(wait=True)
        group.stop()

        if not success:
            rospy.logerr("Home motion planning failed.")
            return

        rospy.loginfo("Home trajectory published. Waiting for physical move...")
        sync.wait_for_move()
        rospy.loginfo("Home reached.")

    rospy.loginfo("Done.")


if __name__ == "__main__":
    try:
        run_trajectory()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nAborted by user.")
