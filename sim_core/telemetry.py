"""
telemetry.py

Contains the Telemetry dataclass, which represents a snapshot
of the vehicle's current state for monitoring and output.
"""




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
    steering: float

    tyre_forces: list[float]

