"""
tyre.py

Contains the Tyre class, which models tyre-road interaction.
"""

from sim_core.math_utils import clamp




class Tyre:
    """
    Represents a vehicle tyre.
    
    Calculates the maximum force that can be transferred
    between tyre and road.
    """

    def __init__(
        self,
        friction_coefficient: float = 1.0,
    ):
        self.friction_coefficient = friction_coefficient

        # Temporarily storing this here (we don't have a suspension model yet)
        self.normal_load: float = 3500.0 # Newtons


    def maximum_grip_force(self) -> float:
        """
        Calculates maximum force available before slipping.

        Returns:
            Maximum longitudinal force in Newtons.
        """

        return self.friction_coefficient * self.normal_load


    def calculate_longitudinal_force(
            self,
            wheel_torque: float,
            wheel_radius: float
    ) -> float:
        """
        Converts wheel torque into road force,
        limited by tyre grip
        """

        if wheel_radius <= 0:
            raise ValueError("Wheel radius must be positive")

        requested_force = wheel_torque / wheel_radius
        grip_limit = self.maximum_grip_force()

        return clamp(
            requested_force,
            -grip_limit,
            grip_limit
        )