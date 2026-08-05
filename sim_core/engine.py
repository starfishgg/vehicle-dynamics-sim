"""
engine.py

Contains the Engine class, which models engine torque production.
"""




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


    def calculate_torque(self, throttle: float) -> float:
        """
        Calculates engine torque output.
        
        Args:
            throttle:
                Accelerator position from 0.0 to 1.0
                
        Returns:
            Torque output in Newton metres.
        """


        throttle = max(0.0, min(throttle, 1.0))

        # This is a simplistic and unrealistic torque calculation.
        # We will improve it to a torque_curve later.
        return self.peak_torque * throttle

    