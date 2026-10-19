"""Pure TFmini configuration and validation helpers.

This module intentionally has no Qt or ROS dependencies, making it easy to
unit-test and debug independently from the graphical interface.
"""

SENSOR_GEOMETRY = {
    "1": {"x": 0.1269, "y": 0.11, "angle_offset": 29.5},
    "2": {"x": 0.1269, "y": -0.11, "angle_offset": 29.5},
    "3": {"x": -0.2633, "y": 0.168, "angle_offset": -150.3},
    "4": {"x": -0.2633, "y": -0.168, "angle_offset": -150.3},
}

FOV_DEGREES = 3.6
WARMUP_SAMPLES = 100
TFMINI_LOST_SIGNAL = 65535.0


def in_pass_range(value, bounds):
    """Return True when a displayed two-decimal value is in an inclusive range."""
    minimum, maximum = (round(bound, 2) for bound in bounds)
    return minimum <= round(value, 2) <= maximum


def is_factory_jig(environment):
    return environment.startswith("Factory Jig")


def pass_range(environment, condition):
    """Return the configured inclusive range for an environment/condition."""
    flat_factory = (45.0, 55.0)
    flat_without_case = (45.0, 54.7)
    cliff_factory = (70.0, 71.0)
    cliff_without_case = (70.2, 70.8)
    wood_flat = (45.0, 55.0)
    wood_cliff = (66.5, 72.6)

    without_case = "Without Case" in environment or "W/O Case" in environment
    if condition == "Flat Surface":
        if without_case:
            return flat_without_case
        return flat_factory if is_factory_jig(environment) else wood_flat
    if without_case:
        return cliff_without_case
    return cliff_factory if is_factory_jig(environment) else wood_cliff
