import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class ValidationResult:
    accepted: bool
    reason: str


def _finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def validate_motion_scalars(
    velocity: float,
    acceleration: float,
    deceleration: float,
    maximums: Sequence[float],
) -> ValidationResult:
    values = [velocity, acceleration, deceleration]
    if not _finite(values):
        return ValidationResult(False, "velocity/acceleration contains non-finite values")
    if any(float(value) <= 0.0 for value in values):
        return ValidationResult(False, "velocity, acceleration and deceleration must be positive")
    names = ["velocity", "acceleration", "deceleration"]
    for name, value, maximum in zip(names, values, maximums):
        if float(value) > float(maximum):
            return ValidationResult(False, f"{name} {value} exceeds limit {maximum}")
    return ValidationResult(True, "ok")


def validate_joint_target(
    target: Sequence[float],
    current: Sequence[float],
    minimum: Sequence[float],
    maximum: Sequence[float],
    max_step_deg: float,
) -> ValidationResult:
    if min(len(target), len(current), len(minimum), len(maximum)) < 6:
        return ValidationResult(False, "joint target/state/limits must contain six values")
    target6 = [float(value) for value in target[:6]]
    current6 = [float(value) for value in current[:6]]
    if not _finite(target6 + current6):
        return ValidationResult(False, "joint target or state contains non-finite values")
    for index, value in enumerate(target6):
        if value < float(minimum[index]) or value > float(maximum[index]):
            return ValidationResult(False, f"J{index + 1}={value:.3f} is outside configured limits")
        delta = abs(value - current6[index])
        if delta > float(max_step_deg):
            return ValidationResult(
                False,
                f"J{index + 1} step {delta:.3f} deg exceeds {max_step_deg:.3f} deg",
            )
    return ValidationResult(True, "ok")


def validate_cartesian_target(
    target: Sequence[float],
    current: Sequence[float],
    workspace_min: Sequence[float],
    workspace_max: Sequence[float],
    max_translation_step_mm: float,
    max_orientation_step_rad: float,
) -> ValidationResult:
    if min(len(target), len(current)) < 6:
        return ValidationResult(False, "Cartesian target/state must contain six values")
    target6 = [float(value) for value in target[:6]]
    current6 = [float(value) for value in current[:6]]
    if not _finite(target6 + current6):
        return ValidationResult(False, "Cartesian target or state contains non-finite values")
    for axis, value, minimum, maximum in zip("XYZ", target6[:3], workspace_min, workspace_max):
        if value < float(minimum) or value > float(maximum):
            return ValidationResult(False, f"{axis}={value:.3f} mm is outside configured workspace")
    translation = math.dist(target6[:3], current6[:3])
    if translation > float(max_translation_step_mm):
        return ValidationResult(
            False,
            f"Cartesian step {translation:.3f} mm exceeds {max_translation_step_mm:.3f} mm",
        )
    orientation_delta = math.sqrt(
        sum(_wrapped_delta(target6[index], current6[index]) ** 2 for index in range(3, 6))
    )
    if orientation_delta > float(max_orientation_step_rad):
        return ValidationResult(
            False,
            f"orientation step {orientation_delta:.3f} rad exceeds {max_orientation_step_rad:.3f} rad",
        )
    return ValidationResult(True, "ok")


def _wrapped_delta(first: float, second: float) -> float:
    return (float(first) - float(second) + math.pi) % (2.0 * math.pi) - math.pi
