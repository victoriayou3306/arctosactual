"""
Trajectory generator for ARCTOS .tap files.

Generates coordinated multi-axis motion as G-code waypoints.
"""

import math


def generate_sweep(
    filename: str = "test.tap",
    steps: int = 50,
    speed: int = 1000,
    x_range: tuple = (-45, 45),
    y_amplitude: float = 10,
    z_amplitude: float = 5,
):
    """
    Generate a sweep trajectory: X sweeps left-to-right while Y and Z oscillate up and down.

    Args:
        filename: Output .tap file path.
        steps: Number of waypoints.
        speed: Motor speed (F value).
        x_range: (min, max) sweep range for X in degrees.
        y_amplitude: Y axis oscillation amplitude in degrees.
        z_amplitude: Z axis oscillation amplitude in degrees.
    """
    with open(filename, "w") as f:
        f.write(f"F{speed}\n")

        for i in range(steps + 1):
            t = i / steps  # 0.0 to 1.0

            # X: linear sweep from min to max
            x = x_range[0] + (x_range[1] - x_range[0]) * t

            # Y: one full sine wave (up-down-up)
            y = y_amplitude * math.sin(2 * math.pi * t)

            # Z: half-frequency cosine (gentle nod)
            z = z_amplitude * math.sin(math.pi * t)

            f.write(f"G90 X{x:.2f} Y{y:.2f} Z{z:.2f} A0 B0 C0\n")

        # Return to home
        f.write("G90 X0 Y0 Z0 A0 B0 C0\n")

    print(f"Generated {steps + 1} waypoints + home → {filename}")


if __name__ == "__main__":
    generate_sweep()
