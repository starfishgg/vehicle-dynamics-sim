"""
math_utils.py

Contains mathematical helper functions used by 
the vehicle simulation
"""

import math




def clamp(value: float, minimum: float, maximum: float) -> float:
    """
    Restricts a value between minimum and maximum.
    """

    return max(minimum, min(value, maximum))


def rotate_local_vector(
        force: float,
        heading: float
) -> tuple[float, float]:
    """
    Converts a force from vehicle coordinates
    into world coordinates.

    Args:
        force:
            Force magnitude in Newtons.

        heading:
            Vehicle heading in degrees.

    Returns:
        Tuple containing world X and Y force.
    """

    radians = math.radians(heading)

    force_x = force * math.cos(radians)
    force_y = force * math.sin(radians)

    return force_x, force_y

