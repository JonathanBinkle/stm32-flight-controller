import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import IntEnum

import serial

MAGIC = 0xDEADBEEF
PORT = "/dev/ttyACM0"
HEADER = struct.Struct("<IBHI")


class TelemetryId(IntEnum):
    """Type of content in telemetry packet. See `enum telemetry_id` in C codebase."""

    RX_SAMPLE = 0
    IMU_SI = 1
    IMU_SAMPLE = 2
    IMU_ANGLES = 3
    ESC_THROTTLES = 4
    FC_OUT = 5
    PID_PITCH = 6
    PID_ROLL = 7
    PID_YAW = 8
    IMU_BIAS = 9


@dataclass(frozen=True)
class PacketDef:
    """A telemetry packet's structure (fields and their data types)."""

    codec: struct.Struct
    fields: tuple[str, ...]


@dataclass
class Packet:
    """A telemetry packet."""

    id: TelemetryId
    seqnum: int
    fields: dict[str, object]


# Definition of packet structures
PID_FIELDS = (
    "prev_time_us",
    "integral_err",
    "prev_actual",
    "Kp",
    "Ki",
    "Kd",
    "pterm",
    "iterm",
    "dterm",
    "desired",
    "actual",
)
PACKET_DEFS = {
    TelemetryId.RX_SAMPLE: PacketDef(
        struct.Struct("<" + "H" * 14),
        (
            "Roll",
            "Pitch",
            "Throttle",
            "Yaw",
            "Ch5",
            "Ch6",
            "Ch7",
            "Ch8",
            "Ch9",
            "Ch10",
            "Ch11",
            "Ch12",
            "Ch13",
            "Ch14",
        ),
    ),
    TelemetryId.IMU_SI: PacketDef(
        struct.Struct("<" + "f" * 6), ("gx", "gy", "gz", "ax", "ay", "az")
    ),
    TelemetryId.IMU_SAMPLE: PacketDef(
        struct.Struct("<" + "f" * 6 + "I"), ("gx", "gy", "gz", "ax", "ay", "az", "dt")
    ),
    TelemetryId.IMU_ANGLES: PacketDef(
        struct.Struct("<" + "f" * 6),
        (
            "fused_pitch",
            "fused_roll",
            "aroll",
            "apitch",
            "groll",
            "gpitch",
        ),
    ),
    TelemetryId.ESC_THROTTLES: PacketDef(
        struct.Struct("<" + "H" * 4),
        ("front_left", "front_right", "back_left", "back_right"),
    ),
    TelemetryId.FC_OUT: PacketDef(
        struct.Struct("<" + "f" * 3),
        ("pitch_pid", "roll_pid", "yaw_pid", "flight_mode"),
    ),
    TelemetryId.PID_PITCH: PacketDef(struct.Struct("<" + "I" + "f" * 10), PID_FIELDS),
    TelemetryId.PID_ROLL: PacketDef(struct.Struct("<" + "I" + "f" * 10), PID_FIELDS),
    TelemetryId.PID_YAW: PacketDef(struct.Struct("<" + "I" + "f" * 10), PID_FIELDS),
    TelemetryId.IMU_BIAS: PacketDef(
        struct.Struct("<" + "f" * 6), ("gx", "gy", "gz", "ax", "ay", "az")
    ),
}


def connect(port: str = PORT, baud: int = 115200, timeout: int = 3):
    """Connects to serial port."""
    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def read_exact(conn: serial.Serial, n: int):
    """Read `n` bytes from serial `conn`ection."""
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.read(n - len(buf))
        if not chunk:
            raise TimeoutError()
        buf.extend(chunk)
    return bytes(buf)


def read_packet(conn: serial.Serial):
    """Read next telemetry packet (magic + header + payload) from `conn`."""

    # Detect packet start through magic
    magic = read_exact(conn, 4)
    while magic != MAGIC.to_bytes(4, "little"):
        magic = magic[1:] + read_exact(conn, 1)

    # Read header
    tid: int
    length: int
    seqnum: int
    tid, length, seqnum = struct.unpack("<BHI", read_exact(conn, 7))

    # Read payload
    payload = read_exact(conn, length)

    # Build packet
    tid = TelemetryId(tid)
    definition = PACKET_DEFS[tid]
    values = PACKET_DEFS[tid].codec.unpack(payload)
    return Packet(tid, seqnum, dict(zip(definition.fields, values)))


def start_reader(
    packet_id: TelemetryId, port: str = PORT, maxlen: int = 10000
) -> tuple[deque[float], dict[str, deque[object]], threading.Lock]:
    """Starts thread that reads packets of type `packet_id` from serial `port`.

    Returns buffers (size `maxlen`) updated by the thread - timestamps, data
    (name:values) - plus thread's lock.
    """

    conn = connect(port)

    fields = PACKET_DEFS[packet_id].fields

    time_buf: deque[float] = deque(maxlen=maxlen)
    data: dict[str, deque[object]] = {f: deque(maxlen=maxlen) for f in fields}

    lock = threading.Lock()

    start = time.monotonic()

    def worker():

        while True:
            pkt = read_packet(conn)

            if pkt.id != packet_id:
                continue

            now = time.monotonic() - start

            with lock:
                time_buf.append(now)

                for f in fields:
                    data[f].append(pkt.fields[f])

    threading.Thread(target=worker, daemon=True).start()

    return time_buf, data, lock
