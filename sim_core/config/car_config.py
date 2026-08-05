

from dataclasses import dataclass




@dataclass
class GearboxConfig:
    ratios: list[float]
    final_drive: float


@dataclass
class EngineConfig:
    idle_rpm: float
    max_rpm: float
    peak_torque: float