"""
Test script to verify J5/J6 differential coupling.

When joint B (q5) moves alone → both Motor 5 and Motor 6 should move in the SAME direction.
When joint C (q6) moves alone → both Motor 5 and Motor 6 should move in OPPOSITE directions.
"""

from convertJoint import joint_to_motor_positions

PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"


def check(label, motor_positions, expected):
    ok = True
    print(f"\n--- {label} ---")
    print(f"  Joint angles → Motor positions:")
    names = ["M1 (X)", "M2 (Y)", "M3 (Z)", "M4 (A)", "M5 (B)", "M6 (C)"]
    for i in range(6):
        status = PASS if abs(motor_positions[i] - expected[i]) < 0.01 else FAIL
        if abs(motor_positions[i] - expected[i]) >= 0.01:
            ok = False
        print(f"  {names[i]}: {motor_positions[i]:>10.2f}°  expected {expected[i]:>10.2f}°  {status}")
    return ok


all_passed = True

# Test 1: Move B (q5) only → both motors should move same direction
print("=" * 60)
print("TEST 1: Move joint B=10°, C=0°")
print("  Expect: M5 and M6 both move (same direction)")
motor = joint_to_motor_positions([0, 0, 0, 0, 10, 0])
expected = [0, 0, 0, 0, (10 + 0) * 67.82, (10 - 0) * 67.82]
all_passed &= check("B=10, C=0", motor, expected)

# Test 2: Move C (q6) only → both motors should move opposite directions
print("\n" + "=" * 60)
print("TEST 2: Move joint B=0°, C=10°")
print("  Expect: M5 and M6 both move (opposite directions)")
motor = joint_to_motor_positions([0, 0, 0, 0, 0, 10])
expected = [0, 0, 0, 0, (0 + 10) * 67.82, (0 - 10) * 67.82]
all_passed &= check("B=0, C=10", motor, expected)

# Test 3: Move both B and C → M6 should cancel out
print("\n" + "=" * 60)
print("TEST 3: Move joint B=10°, C=10°")
print("  Expect: M5 moves (double), M6 stays at 0 (cancels)")
motor = joint_to_motor_positions([0, 0, 0, 0, 10, 10])
expected = [0, 0, 0, 0, (10 + 10) * 67.82, (10 - 10) * 67.82]
all_passed &= check("B=10, C=10", motor, expected)

# Test 4: Move B and C opposite → M5 should cancel out
print("\n" + "=" * 60)
print("TEST 4: Move joint B=10°, C=-10°")
print("  Expect: M5 stays at 0 (cancels), M6 moves (double)")
motor = joint_to_motor_positions([0, 0, 0, 0, 10, -10])
expected = [0, 0, 0, 0, (10 - 10) * 67.82, (10 + 10) * 67.82]
all_passed &= check("B=10, C=-10", motor, expected)

print("\n" + "=" * 60)
print(f"\nOverall: {PASS if all_passed else FAIL}")
