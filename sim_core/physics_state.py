"""
physics_state.py

Contains the PhysicsState dataclass, which tracjs the
current physical state of a vehicle during the simulation.
"""

from dataclasses import dataclass
import math




@dataclass
class PhysicsState:
    """
    Represents the current motion state of a vehicle.
    """

    mass: float = 1500.0 # kg

    # Position (metres)
    position_x: float = 0.0
    position_y: float = 0.0

    # Velocity vector (m/s)
    velocity_x: float = 0.0
    velocity_y: float = 0.0

    # Acceleration vector
    acceleration_x: float = 0.0
    acceleration_y: float = 0.0

    # Vehicle orientation (degrees)
    heading: float = 0.0

    force_x: float = 0.0
    force_y: float = 0.0


    @property
    def speed(self) -> float:
        return math.sqrt(
            self.velocity_x ** 2 +
            self.velocity_y ** 2
        )

    @property
    def acceleration(self) -> float:
        return math.sqrt(
            self.acceleration_x ** 2 +
            self.acceleration_y ** 2
        )
    

    def add_force(
            self,
            force_x: float,
            force_y: float
    ) -> None:
        
        self.force_x += force_x
        self.force_y += force_y


    def update(self, dt: float) -> None:

        self.acceleration_x = self.force_x / self.mass
        self.acceleration_y = self.force_y / self.mass

        self.velocity_x += self.acceleration_x * dt
        self.velocity_y += self.acceleration_y * dt

        # Euler integration, we may improve this later.
        self.position_x += self.velocity_x * dt
        self.position_y += self.velocity_y * dt

        self.force_x = 0.0
        self.force_y = 0.0