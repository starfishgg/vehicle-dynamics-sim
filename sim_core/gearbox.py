"""
gearbox.py

Contains the Gearbox class, which models transmission behaviour
and torque multiplication.
"""




class Gearbox:

    def __init__(
        self,
        current_gear: int=1,
        gear_ratios: list[float] | None = None,
        final_drive_ratio: float = 3.90
    ):
        self.current_gear = current_gear

        if gear_ratios is None:
            self.gear_ratios = [
                3.50,
                2.10,
                1.40,
                1.00,
                0.80
            ]
        else:
            self.gear_ratios = gear_ratios

        self.final_drive_ratio = final_drive_ratio


    def get_total_ratio(self) -> float:
        """
        Returns the combined gear ratio and final-drive ratio.
        """

        return self.get_current_ratio() * self.final_drive_ratio

    
    def get_current_ratio(self) -> float:
        return self.gear_ratios[self.current_gear - 1]


    def multiply_torque(self, engine_torque: float) -> float:
        return engine_torque * self.get_current_ratio()

    
