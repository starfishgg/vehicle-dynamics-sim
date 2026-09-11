"""
drivetrain.py

Handles transferring engine torque through the gearbox
and distributing it to driven wheels.
"""

from sim_core.differential import Differential
from sim_core.wheel import Wheel
from sim_core.gearbox import Gearbox
from sim_core.settings import REAR_LEFT, REAR_RIGHT

import math




class Drivetrain:

    def __init__(
        self,
        gearbox: Gearbox,
        differential: Differential,
        driven_wheels_indices: list[int] = [REAR_LEFT, REAR_RIGHT]
    ):

        self.gearbox = gearbox
        self.driven_wheels_indices = driven_wheels_indices
        self.differential = differential
        self.efficiency = 0.9


    def calculate_engine_rpm(
            self,
            wheels: list[Wheel]
    ) -> float:
        """
        Calculates engine RPM from driven wheel speed.
        """

        if not self.driven_wheels_indices:
            raise ValueError("Drivetrain has no driven wheels")

        driven_wheel = wheels[self.driven_wheels_indices[0]]

        wheel_rpm = (
            driven_wheel.angular_velocity * 60.0 / (2.0 * math.pi)
        )

        engine_rpm = (
            wheel_rpm * self.gearbox.get_total_ratio()
        )

        return engine_rpm


    def apply_torque(
            self, 
            engine_torque: float, 
            wheels: list[Wheel]
    ) -> None:

        if not self.driven_wheels_indices:
            raise ValueError("Drivetrain has no driven wheels")
        
        wheel_torque = (
            self.gearbox.get_total_ratio()
            * engine_torque
            * self.efficiency
        )

        driven_wheels = [
            wheels[i]
            for i in self.driven_wheels_indices
        ]

        self.differential.distribute_torque(
            wheel_torque,
            driven_wheels
        )



