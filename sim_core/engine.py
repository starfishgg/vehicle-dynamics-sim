"""
engine.py

Contains the Engine class, which models engine torque production.
"""


from sim_core.math_utils import clamp




class Engine:
    """
    Represents an internal combustion engine.
    
    The engine converts throttle input and RPM into torque output.
    """

    def __init__(
            self,
            idle_rpm: float = 900.0,
            max_rpm: float = 7000.0,
            peak_torque: float = 300.0
    ):
        self.idle_rpm = idle_rpm
        self.max_rpm = max_rpm
        self.rpm = idle_rpm

        # Maximum engine torque
        self.peak_torque = peak_torque
        self.current_torque = 0.0


    def set_rpm(self, rpm: float) -> None:
        self.rpm = clamp(rpm, self.idle_rpm, self.max_rpm)


    def calculate_torque(self, throttle: float) -> float:
        """
        Calculates engine torque output based on throttle position and RPM.
        
        Args:
            throttle:
                Accelerator position from 0.0 to 1.0
                
        Returns:
            Torque output in Newton metres.
        """


        throttle = clamp(throttle, 0.0, 1.0)

        # Simple torque curve:
        # - Lower torque at idle
        # - Peak torque around 4500 RPM
        # - Torque falls away towards redline
        
        if self.rpm <= 1500:
            torque_factor = 0.70

        elif self.rpm <= 4500:
            torque_factor = 0.70 + (
                0.30 * (self.rpm - 1500) / 3000
            )

        else:
            torque_factor = 1.0 - (
                0.25 * (self.rpm - 4500) / 2500
            )

        torque_factor = clamp(torque_factor, 0.0, 1.0)

        self.current_torque = (
            self.peak_torque
            * torque_factor
            * throttle
        )
        
        return self.current_torque

    