import os
import re
from typing import List

# Parameters Section
# -------------------

# Gear ratios for each motor (joint angle to motor angle)
gear_ratios: List[float] = [
    13.5,    # X (J1) - Belt reduction
    150,     # Y (J2) - Cycloidal
    150,     # Z (J3) - Cycloidal
    48,      # A (J4) - Compound Planetary
    67.82,   # B (J5) - Compound Planetary
    67.82,   # C (J6) - Compound Planetary
]


# Direction inversion for each motor
invert_direction: List[bool] = [
    True,   # X (J1)
    True,   # Y (J2)
    False,  # Z (J3)
    False,  # A (J4)
    False,  # B (J5)
    False,  # C (J6)
]

# Initialize last motor positions and last joint angles for position tracking
last_motor_positions: List[float] = [0.0] * 6
last_joint_angles: List[float] = [0.0] * 6

# -------------------


def joint_to_motor_positions(joint_angles: List[float]) -> List[float]:
    """
    Converts joint angles (degrees) to motor positions (degrees),
    applying gear ratios, direction inversion, and differential coupling for J5/J6.

    Args:
        joint_angles: List of 6 joint angles [q1, q2, q3, q4, q5, q6] in degrees.

    Returns:
        List of 6 motor positions in degrees.
    """
    q1, q2, q3, q4, q5, q6 = joint_angles

    motor_positions = [
        q1 * gear_ratios[0],
        q2 * gear_ratios[1],
        q3 * gear_ratios[2],
        q4 * gear_ratios[3],
        (q5 + q6) * gear_ratios[4],  # Differential coupling
        (q5 - q6) * gear_ratios[5],  # Differential coupling
    ]

    # Apply direction inversion
    for i in range(6):
        if invert_direction[i]:
            motor_positions[i] = -motor_positions[i]

    return motor_positions


def calculate_crc(data: List[int]) -> int:
    """Calculates a simple CRC value for a list of integers."""
    return sum(data) & 0xFF


def convert_to_can_message(axis_id: int, speed: int, motor_position: float) -> str:
    """
    Converts a motor position into a CAN message string.

    Args:
        axis_id: The ID of the motor axis (1-6).
        speed: The speed for the motor axis.
        motor_position: The target motor position in degrees (already includes
                        gear ratio, coupling, and inversion).

    Returns:
        A string representing the CAN message.
    """
    can_id = format(axis_id, "02X")
    speed_hex = format(speed, "04X")

    # Calculate relative position from last position
    rel_position = int((motor_position - last_motor_positions[axis_id - 1]) * 100)

    # Handle signed 24-bit integer using two's complement
    rel_position_hex = format(rel_position & 0xFFFFFF, "06X")

    # Update last motor position
    last_motor_positions[axis_id - 1] = motor_position

    return can_id + "F5" + speed_hex + "02" + rel_position_hex


def process_tap_files() -> None:
    """
    Processes all .tap files in the script directory, converting joint angles
    to motor positions and outputting CAN messages to .txt files.
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

                    if line.startswith("G90") or line.startswith("G91"):
                        values = [
                            float(value) if "." in value else int(value)
                            for value in re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", line)
                        ]

                        if len(values) >= 7:
                            if line.startswith("G91"):
                                # Relative: add deltas to current joint angles
                                joint_angles = [
                                    last_joint_angles[i] + float(values[i + 1])
                                    for i in range(6)
                                ]
                            else:
                                # Absolute
                                joint_angles = [float(v) for v in values[1:7]]

                            last_joint_angles[:] = joint_angles
                            motor_positions = joint_to_motor_positions(joint_angles)

                            for axis_id in range(1, 7):
                                can_message = convert_to_can_message(
                                    axis_id, speed, motor_positions[axis_id - 1]
                                )
                                crc = calculate_crc(
                                    [
                                        int(can_message[i : i + 2], 16)
                                        for i in range(0, len(can_message), 2)
                                    ]
                                )
                                can_message_with_crc = can_message + format(crc, "02X")
                                output_file.write(can_message_with_crc + "\n")
                                print(
                                    f"Converted g-code line to CAN message: {can_message_with_crc}"
                                )


if __name__ == "__main__":
    process_tap_files()
