

from dataclasses import dataclass




@dataclass
class Telemetry:

    speed_kmh: float
    position_x: float
    position_y: float

    acceleration: float

    engine_rpm: float
    engine_torque: float

    gear: int

    throttle: float

    tyre_forces: list[float]

