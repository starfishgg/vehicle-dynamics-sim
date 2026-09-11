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
        friction_coefficient: float = 1.0
    ):
        self.friction_coefficient = friction_coefficient

        # Temporarily storing this here until we have suspension
        self.normal_load: float = 3500.0 # Newtons

        # Slip ratio at which maximum grip is reached.
        self.peak_slip_ratio: float = 0.10

        # Beyong this point, the tyre has lost all longitudinal grip.
        self.maximum_slip_ratio: float = 0.20

        self.sliding_friction_coefficient = 0.8


    def maximum_grip_force(self) -> float:
        """
        Calculates maximum force available before slipping.

        Returns:
            Maximum longitudinal force in Newtons.
        """

        return self.friction_coefficient * self.normal_load


    def calculate_longitudinal_force(
            self,
            slip_ratio: float
    ) -> float:
        """
        Calculates longitudinal tyre force from slip ratio.

        Args:
            slip_ratio:
                Difference between wheel rotational speed
                and vehicle speed.

        Returns:
            Longitudinal tyre force in Newtons.
        """

        maximum_force = self.maximum_grip_force()

        if slip_ratio == 0.0:
            return 0.0

        slip_sign = 1.0 if slip_ratio > 0.0 else -1.0
        slip = abs(slip_ratio)

        if slip <= self.peak_slip_ratio:

            force = (maximum_force * slip / self.peak_slip_ratio)

        else:

            force = maximum_force * self.sliding_friction_coefficient

        return slip_sign * force


    def calculate_lateral_force(
            self,
            slip_angle: float,
            wheel_speed: float
    ) -> float:
        """
        Calculate lateral tyre force from slip angle.

        Args:
            slip_angle:
                Angle between the drection the tyre is pointing and the direction it is travelling, in degrees.

        Returns:
            Lateral tyre force in Newtons.
        """

        # Maximum lateral force currently available from the tyre.
        maximum_force = self.maximum_grip_force()

        # A stationary tyre does not generate meaningful cornering 
        # force from its steering angle alone.
        if abs(wheel_speed) < 0.1:
            return 0.0

        # Simple cornering stiffness.
        # This determines how much force is generated per degree
        # of slip angle.
        cornering_stiffness = 100.0 # N per degree

        lateral_force = cornering_stiffness * slip_angle

        # A real tyre cannot generate unlimited force.
        lateral_force = clamp(
            lateral_force,
            -maximum_force,
            maximum_force
        )

        return lateral_force

    
