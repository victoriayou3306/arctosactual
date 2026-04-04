import os
import time
from typing import List

import can


def parse_can_message(line: str) -> can.Message:
    """
    Parses a single line from a file representing a CAN message.

    Args:
        line: A string representing a CAN message in the format "ID DATA".

    Returns:
        A `can.Message` object with the parsed arbitration ID and data bytes.
    """
    parts = line.split(" ")
    arbitration_id = int(parts[0][:2], 16)
    data = [int(parts[0][i : i + 2], 16) for i in range(2, len(parts[0]), 2)] + [
        int(byte, 16) for byte in parts[1:]
    ]
    return can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)


def calculate_crc(arbitration_id: int, status: int) -> int:
    """
    Calculates a simple CRC value based on the arbitration ID and status.

    Args:
        arbitration_id: The arbitration ID of the CAN message.
        status: An arbitrary status value.

    Returns:
        An integer representing the calculated CRC value.
    """
    return (arbitration_id + 0xF4 + status) & 0xFF


def adjust_speeds_within_packet(messages: List[can.Message]) -> None:
    """
    Adjusts the speeds within a chunk of CAN messages based on the average speed.

    Args:
        messages: A list of `can.Message` objects representing a coordinated chunk.
    """
    speeds = [(msg.data[3] << 8) + msg.data[4] for msg in messages]
    reference_speed = sum(speeds) // len(speeds)
    if reference_speed == 0:
        return
    for msg in messages:
        speed = (msg.data[3] << 8) + msg.data[4]
        adjusted_speed = int((speed / reference_speed) * reference_speed)
        msg.data[3] = (adjusted_speed >> 8) & 0xFF
        msg.data[4] = adjusted_speed & 0xFF


def send_chunk_and_wait(
    bus: can.interface.Bus,
    chunk: List[can.Message],
    timeout: float = 5.0,
    status_byte_index: int = 1,
    expected_status: int = 0x02,
    skip_ids: set = None,
) -> bool:
    expected_ids = {msg.arbitration_id for msg in chunk}
    if skip_ids:
        expected_ids -= skip_ids
    confirmed_ids: set = set()
    error_ids: set = set()

    for msg in chunk:
        bus.send(msg)
        data_bytes = ", ".join(f"0x{b:02X}" for b in msg.data)
        print(f"  → Sent:     ID=0x{msg.arbitration_id:02X}  data=[{data_bytes}]")

    print(f"  Waiting for ACK from motors: {[hex(i) for i in sorted(expected_ids)]}")
    if skip_ids:
        print(f"  Skipping ACK wait for motors: {[hex(i) for i in sorted(skip_ids)]}")

    deadline = time.time() + timeout
    while time.time() < deadline:
        remaining = deadline - time.time()
        received_msg = bus.recv(timeout=min(remaining, 0.1))

        if received_msg is None:
            continue

        data_bytes = ", ".join(f"0x{b:02X}" for b in received_msg.data)
        print(f"  ← Received: ID=0x{received_msg.arbitration_id:02X}  data=[{data_bytes}]")

        if received_msg.arbitration_id in expected_ids and len(received_msg.data) > status_byte_index:
            status = received_msg.data[status_byte_index]

            if status == expected_status:
                confirmed_ids.add(received_msg.arbitration_id)
                print(f"    ✓ Motor 0x{received_msg.arbitration_id:02X} confirmed complete.")

            elif status == 0x00:
                error_ids.add(received_msg.arbitration_id)
                print(f"    ✗ Motor 0x{received_msg.arbitration_id:02X} returned ERROR (status=0x00) — command rejected.")

        # Treat errored motors as done (failed) so they don't block the timeout
        if (confirmed_ids | error_ids) == expected_ids:
            if error_ids:
                print(f"  ✗ Chunk completed with errors on motors: {[hex(i) for i in sorted(error_ids)]}\n")
                return False
            print(f"  ✓ All motors in chunk confirmed. Proceeding.\n")
            return True

    print(f"  ✗ Timeout! No ACK from: {[hex(i) for i in sorted(expected_ids - confirmed_ids - error_ids)]}\n")
    return False


def send_six_dof_sequence(
    bus: can.interface.Bus,
    all_messages: List[can.Message],
    chunk_size: int = 6,
    timeout_per_chunk: float = 5.0,
    status_byte_index: int = 1,
    skip_ids: set = None,
) -> None:
    """
    Sends 6 motor messages in sequential chunks, waiting for full ACK
    from each chunk before sending the next.

    Args:
        bus:               The CAN bus instance.
        all_messages:      Flat list of 6 can.Message objects (one per motor).
        chunk_size:        How many motors to command per chunk (default 6).
        timeout_per_chunk: Seconds to wait for each chunk's ACKs.
        status_byte_index: Index in response data[] that holds the status byte.
        skip_ids:          Set of motor arbitration IDs to send but not wait on.
    """
    chunks = [all_messages[i : i + chunk_size] for i in range(0, len(all_messages), chunk_size)]

    for idx, chunk in enumerate(chunks):
        ids = [hex(m.arbitration_id) for m in chunk]
        print(f"── Chunk {idx + 1}/{len(chunks)}  motors={ids} ──")
        adjust_speeds_within_packet(chunk)
        success = send_chunk_and_wait(
            bus,
            chunk,
            timeout=timeout_per_chunk,
            status_byte_index=status_byte_index,
            skip_ids=skip_ids,
        )
        if not success:
            print(f"Aborting sequence at chunk {idx + 1} due to timeout.")
            return

    print("All chunks completed successfully.")


def main() -> None:
    """
    Main function to read CAN messages from a .txt file, send them through
    a CAN bus in sequential chunks, and wait for motor ACKs between chunks.
    """
    script_directory = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_directory, "test.txt")

    bus = can.interface.Bus(interface="slcan", channel="/dev/ttyACM0", bitrate=500000)

    with open(file_path, "r") as file:
        lines = [l.strip() for l in file.readlines() if l.strip()]

    # Each group of 6 lines = one full arm pose
    pose_sets = [lines[i : i + 6] for i in range(0, len(lines), 6)]

    for pose_idx, pose_lines in enumerate(pose_sets):
        print(f"\n══════ Pose {pose_idx + 1}/{len(pose_sets)} ══════")
        messages = [parse_can_message(line) for line in pose_lines]

        send_six_dof_sequence(
            bus,
            messages,
            chunk_size=6,
            timeout_per_chunk=15.0,
            status_byte_index=1,
            skip_ids={0xb},        # remove once motor 5 hardware is verified
        )

    bus.shutdown()
    print("Done.")


if __name__ == "__main__":
    main()