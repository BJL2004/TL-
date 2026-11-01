from enum import IntEnum
from typing import Iterable, Sequence


class Register(IntEnum):
    FW_VERSION = 1001
    STATUS = 1085
    CURRENT_LIMIT = 1095
    CURRENT = 1105
    FORCE_LIMIT = 1115
    FORCE = 1120
    SPEED = 1125
    POSITION_TARGET = 1135
    POSITION = 1145
    STALL_SPEED = 1175
    STALL_CURRENT = 1185
    STOP_AFTER_PERIOD = 1195
    RETRY_AFTER_PERIOD = 1205


class FingerStatus(IntEnum):
    OPENING = 0
    CLOSING = 1
    POSITION_REACHED = 2
    OVER_CURRENT = 3
    FORCE_REACHED = 4
    STUCK = 5


FAULT_STATUSES = {FingerStatus.OVER_CURRENT, FingerStatus.STUCK}
MOVING_STATUSES = {FingerStatus.OPENING, FingerStatus.CLOSING}


def classify_operation(
    raw_statuses: Sequence[int],
    operation: str,
    contact_fingers: Iterable[int] = range(5),
) -> str:
    try:
        statuses = [FingerStatus(int(value)) for value in raw_statuses]
    except ValueError:
        return "fault_unknown_status"
    if any(status in FAULT_STATUSES for status in statuses):
        return "fault"
    if any(status in MOVING_STATUSES for status in statuses):
        return "moving"
    if operation == "open":
        return "open_reached" if all(status == FingerStatus.POSITION_REACHED for status in statuses) else "fault"
    if operation == "grasp":
        contact = any(statuses[index] == FingerStatus.FORCE_REACHED for index in contact_fingers)
        return "contact" if contact else "no_contact"
    return "idle"


def release_toward_open(
    current: Sequence[int],
    open_position: Sequence[int],
    ratio: float,
) -> list[int]:
    bounded_ratio = max(0.0, min(1.0, float(ratio)))
    return [
        int(round(float(now) + (float(open_value) - float(now)) * bounded_ratio))
        for now, open_value in zip(current, open_position)
    ]


def validate_register_values(values: Sequence[int], count: int, name: str) -> list[int]:
    if len(values) != count:
        raise ValueError(f"{name} must contain {count} values")
    normalized = [int(value) for value in values]
    if any(value < 0 or value > 65535 for value in normalized):
        raise ValueError(f"{name} values must be in uint16 range")
    return normalized
