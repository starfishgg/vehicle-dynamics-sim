"""
wheel.py

Contains the Wheel class, whichmodels a vehicle wheel and
converts drivetrain torque into tyre forces.
"""

from sim_core.tyre import Tyre




class Wheel:
    """
    Represents a driven vehicle wheel.
    
    Conterts torque into a force applied through the tyre.
    """

    def __init__(
        self,
        name: str,
        radius: float = 0.34,
        tyre: Tyre | None = None
    ):
        self.radius = radius # in metres
        self.tyre = tyre
        self.name = name

        # torque applied by drivetrain
        self.drive_torque = 0.0

        self.last_longitudinal_force = 0.0


    def calculate_force(self) -> float:

        if self.tyre is None:
            raise ValueError("Wheel has no tyre attached")

        self.last_longitudinal_force = self.tyre.calculate_longitudinal_force(
            self.drive_torque,
            self.radius
        )

        return self.last_longitudinal_force


    def apply_drive_torque(self, torque: float) -> None:
        self.drive_torque = torque


    def clear_drive_torque(self) -> None:
        self.drive_torque = 0.0

        