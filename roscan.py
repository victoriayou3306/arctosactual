import os
import re
import math

# ROS imports — gracefully skip if ROS not available
try:
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[roscan] ROS not found — running without joint state publishing")

# Parameters Section
# -------------------

# Gearbox ratios for each motor
gear_ratios = [6.75, 75, 75, 24, 33.91, 33.91]

# Direction inversion for each motor (True/False)
invert_direction = [True, True, False, False, False, False]

# ROS joint names — must match your URDF exactly
JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]

# Initialize zero positions and last positions
initial_positions = [0] * 6
last_positions    = [0] * 6

# Current joint angles in radians (updated as commands are converted)
current_joint_angles = [0.0] * 6

# -------------------

# Init ROS node + publisher once at module level
_ros_pub = None

def _init_ros():
    global _ros_pub
    if not ROS_AVAILABLE or _ros_pub is not None:
        return
    try:
        if not rospy.core.is_initialized():
            rospy.init_node("arctos_roscan_bridge", anonymous=True, disable_signals=True)
        _ros_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        rospy.sleep(0.3)  # let publisher register
        print("[roscan] ROS joint_states publisher ready → RViz will update live")
    except Exception as e:
        print(f"[roscan] ROS init failed: {e}")
        _ros_pub = None


def _publish_joint_states():
    """Publish current joint angles to /joint_states so RViz updates."""
    if _ros_pub is None:
        return
    try:
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name     = JOINT_NAMES
        js.position = list(current_joint_angles)
        js.velocity = [0.0] * 6
        js.effort   = [0.0] * 6
        _ros_pub.publish(js)
    except Exception as e:
        print(f"[roscan] publish error: {e}")


def calculate_crc(data):
    crc = sum(data) & 0xFF
    return crc


def convert_to_can_message(axis_id, speed, position, gear_ratio, invert_direction=False):
    can_id = format(axis_id+7, '02X')
    speed_hex = format(speed, '04X')

    # Calculate relative position based on the initial position
    rel_position = int((position * gear_ratio - initial_positions[axis_id - 1]) * 100)

    # Handle signed 24-bit integer using two's complement representation
    rel_position_hex = format(rel_position & 0xFFFFFF, '06X')

    # Update last_position for the axis
    last_positions[axis_id - 1] = position * gear_ratio

    # ── ROS: update joint angle and publish ──────────────────────────────────
    joint_idx = axis_id - 1
    # position here is in degrees (from G-code), convert to radians for ROS
    current_joint_angles[joint_idx] = math.radians(position)
    if any(a != 0.0 for a in current_joint_angles):
        _publish_joint_states()
    # ─────────────────────────────────────────────────────────────────────────

    return can_id + 'F4' + speed_hex + '02' + rel_position_hex


def console():
    import can
    # Get user input from text input field
    user_input = console_input.get().strip()

    try:
        msg_data = bytes.fromhex(user_input)
        msg = can.Message(data=msg_data, is_extended_id=False)
        try:
            with can.interface.Bus(interface="socketcan", channel="can0") as bus:
                bus.send(msg)
                print("Message sent successfully.")
        except Exception as e:
            print(f"Error sending message: {e}")
    except ValueError:
        print("Invalid input format. Please enter a valid hex message.")


def process_tap_files():
    _init_ros()  # start ROS publisher before processing

    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_filename  = os.path.join(script_dir, "jog.tap")
    output_filename = os.path.join(script_dir, "jog.txt")

    with open(input_filename, 'r') as input_file, open(output_filename, 'w') as output_file:
        speed = 0

        for line in input_file:
            speed_match = re.search(r'F(\d+)', line)
            if speed_match:
                try:
                    speed = int(speed_match.group(1))
                except ValueError:
                    continue

            if line.startswith("G90"):
                values = [float(value) if '.' in value else int(value)
                          for value in re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', line)]

                if len(values) >= 7:
                    for axis_id, position in enumerate(values[1:7], start=1):
                        gear_ratio  = gear_ratios[axis_id - 1]
                        invert_dir  = invert_direction[axis_id - 1]
                        can_message = convert_to_can_message(
                            axis_id, speed, position, gear_ratio, invert_dir
                        )
                        crc = calculate_crc(
                            [int(can_message[i:i+2], 16) for i in range(0, len(can_message), 2)]
                        )
                        can_message_with_crc = can_message + format(crc, '02X')
                        output_file.write(can_message_with_crc + '\n')
                        print(f"Converted g-code line to CAN message: {can_message_with_crc}")


if __name__ == "__main__":
    process_tap_files()
