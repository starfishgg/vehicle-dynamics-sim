"""
drivetrain.py

Handles transferring engine torque through the gearbox
and distributing it to driven wheels.
"""

from sim_core.differential import Differential
from sim_core.wheel import Wheel
from sim_core.settings import *




class Drivetrain:

    def __init__(
        self,
        gearbox,
        differential: Differential,
        driven_wheels_indices: list[int] = [REAR_LEFT, REAR_RIGHT]
    ):

        self.gearbox = gearbox
        self.driven_wheels_indices = driven_wheels_indices
        self.differential = differential
        self.efficiency = 0.9


    def apply_torque(
            self, 
            engine_torque: float, 
            wheels: list[Wheel]
    ) -> None:

        if not self.driven_wheels_indices:
            raise ValueError("Drivetrain has no driven wheels")
        
        wheel_torque = self.gearbox.multiply_torque(engine_torque)
        wheel_torque *= self.efficiency

        driven_wheels = [
            wheels[i]
            for i in self.driven_wheels_indices
        ]

        self.differential.distribute_torque(
            wheel_torque,
            driven_wheels
        )



