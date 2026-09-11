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
        local_x: float,
        local_y: float,
        heading: float
) -> tuple[float, float]:
    """
    Converts a vector from vehicle-local coordinates
    into world coordinates.

    Args:
        local_x:
            Vector component in the vehicle's forward direction.

        local_y:
            Vector component in the vehicle's sideways direction.

        heading:
            Vehicle heading in degrees.

    Returns:
        A tuple containing the  world X and Y components.
    """

    heading_radians = math.radians(heading)

    world_x = (
        local_x * math.cos(heading_radians)
        - local_y * math.sin(heading_radians)
    )

    world_y = (
        local_x * math.sin(heading_radians)
        + local_y * math.cos(heading_radians)
    )

    return world_x, world_y

