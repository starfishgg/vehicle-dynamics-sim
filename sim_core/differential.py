"""
differentia.py

Contains the Differential class, which distributes
torque between driven wheels.
"""

from sim_core.wheel import Wheel




class Differential:
    """
    Simple open differential.
    
    Splits torque evenly between driven wheels.
    """

    def __init__(self):
        pass
    

    def distribute_torque(
        self,
        torque: float,
        wheels: list[Wheel]
    ) -> None:
        """
        Splits torque evenly between driven wheels.
        """


        if not wheels:
            raise ValueError("Differential has no connected wheels")

        wheel_torque = torque / len(wheels)

        for wheel in wheels:
            # This is a temporary simplification
            wheel.apply_drive_torque(wheel_torque)

