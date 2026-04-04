import os
import re
from typing import List

# Parameters Section
# -------------------

# Gearbox ratios for each motor
gear_ratios: List[float] = [
    13.5/2,    # X (J1) - Belt reduction
    150/2,     # Y (J2) - Cycloidal
    150/2,     # Z (J3) - Cycloidal
    48/2,      # A (J4) - Compound Planetary
    67.82/2,   # B (J5) - Compound Planetary
    67.82/2,   # C (J6) - Compound Planetary
]

# Direction inversion for each motor (True/False)
invert_direction: List[bool] = [True, False, True, False, True, False] # Set True for motors where direction should be inverted

# Initialize zero positions and last positions
initial_positions: List[int] = [0] * 6
last_positions: List[float] = [0] * 6
current_joint_positions: List[float] = [0] * 6

# -------------------

def apply_differential_mix(angles_rad):
    """
    Convert logical joint angles [B, C] into motor commands [M5, M6]
    for a bevel gear differential.
    Joints 0-3 (J1-J4) pass through unchanged.
    """
    mixed = list(angles_rad)  # copy so we don't mutate the original

    b = angles_rad[4]  # joint 5 (B) — 0-indexed
    c = angles_rad[5]  # joint 6 (C)

    mixed[4] = (b + c) / 2.0   # M5
    mixed[5] = (-b + c) / 2.0   # M6
    # rospy.loginfo(f"+-Differential mix: B={math.degrees(b):.2f} C={math.degrees(c):.2f} → M5={math.degrees(mixed[4]):.2f} M6={math.degrees(mixed[5]):.2f}")

    return mixed


def calculate_crc(data: List[int]) -> int:
    """
    Calculates a simple CRC (Cyclic Redundancy Check) value for a list of integers.

    Args:
        data: A list of integers representing the data to calculate the CRC for.

    Returns:
        An integer representing the calculated CRC value.
    """
    crc = sum(data) & 0xFF
    return crc


def convert_to_can_message(
    axis_id: int,
    speed: int,
    position: float,
    gear_ratio: float,
    invert_direction: bool = False,
) -> str:
    """
    Converts motion parameters into a CAN message string.

    TODO: #1 invert_direction is not used in this function. Implement the logic to invert the direction of the motor axis.

    Args:
        axis_id: The ID of the motor axis.
        speed: The speed for the motor axis.
        position: The target position for the motor axis.
        gear_ratio: The gear ratio for the motor axis.
        invert_direction: If True, the direction of the motor axis is inverted.

    Returns:
        A string representing the CAN message.
    """
    can_id = format(axis_id + 6, "02X")
    speed_hex = format(speed, "04X")

    # Calculate relative position based on the last position
    pos = position
    if invert_direction:
        pos *= -1
    rel_position = int((pos * gear_ratio - last_positions[axis_id - 1]) * 100)

    # Handle signed 24-bit integer using two's complement representation
    rel_position_hex = format(rel_position & 0xFFFFFF, "06X")

    # Update last_position for the axis
    last_positions[axis_id - 1] = pos * gear_ratio

    return can_id + "F4" + speed_hex + "02" + rel_position_hex


def process_tap_files() -> None:
    """
    Processes all .tap files in the script directory, converting them into .txt files containing CAN messages.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))

    for filename in os.listdir(script_dir):
        if filename.endswith(".tap"):
            input_filename = os.path.join(script_dir, filename)
            output_filename = os.path.join(
                script_dir, os.path.splitext(filename)[0] + ".txt"
            )

            with open(input_filename, "r") as input_file, open(
                output_filename, "w"
            ) as output_file:
                speed = 0

                for line in input_file:
                    speed_match = re.search(r"F(\d+)", line)
                    if speed_match:
                        try:
                            speed = int(speed_match.group(1))
                        except ValueError:
                            continue

                    is_g90 = line.startswith("G90")
                    is_g91 = line.startswith("G91")

                    if is_g90 or is_g91:
                        values = [
                            float(value) if "." in value else int(value)
                            for value in re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", line)
                        ]

                        if len(values) >= 7:
                            deltas = list(values[1:7])

                            if is_g90:
                                axis_positions = deltas
                            else:  # G91
                                axis_positions = [
                                    current_joint_positions[i] + deltas[i]
                                    for i in range(6)
                                ]

                            # Update joint-space tracker
                            for i in range(6):
                                current_joint_positions[i] = axis_positions[i]

                            axis_positions = apply_differential_mix(axis_positions)

                            for axis_id, position in enumerate(axis_positions, start=1):
                                gear_ratio = gear_ratios[axis_id - 1]
                                invert_dir = invert_direction[axis_id - 1]
                                can_message = convert_to_can_message(
                                    axis_id, speed, position, gear_ratio, invert_dir
                                )
                                crc = calculate_crc(
                                    [int(can_message[i:i+2], 16) for i in range(0, len(can_message), 2)]
                                )
                                can_message_with_crc = can_message + format(crc, "02X")
                                output_file.write(can_message_with_crc + "\n")
                                print(f"Converted g-code line to CAN message: {can_message_with_crc}")


if __name__ == "__main__":
    process_tap_files()