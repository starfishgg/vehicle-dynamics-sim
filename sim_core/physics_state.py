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

    # Rotational motion around the vertical (Z) axis.
    yaw_rate: float = 0.0 # in degrees/sec
    yaw_acceleration: float = 0.0 # in degrees/sec²
    yaw_inertia: float = 2500.0 # kg m²

    # Forces acting on the vehicle
    force_x: float = 0.0
    force_y: float = 0.0

    # Rotational moment around the vertical (Z) axis.
    moment_z: float = 0.0

    # Most recent yaw moment, preserved for telemetry.
    last_moment_z: float = 0.0


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


    def add_wheel_moment(
            self,
            position_x: float,
            position_y: float,
            force_x: float,
            force_y: float
    ) -> None:
        """
        Adds the yaw moment produuced by a wheel force.
        
        Args:
            position_x: 
                Wheel's forwards/rearward position relative to the vehicle's centre of mass.

            position_y: 
                Wheel's left/right position relative to the vehicle's centre of mass.

            force_x: 
                Force acting in the vehicle's forward direction.

            force_y: 
                Force acting sideways.
        """

        wheel_moment = (
            position_x * force_y
            - position_y * force_x
        )

        self.moment_z += wheel_moment


    def get_wheel_velocity(
            self,
            wheel_position_x: float,
            wheel_position_y: float
    ) -> tuple[float, float]:
        """
        Calculates the velocity at a wheel's position in vehicle-local coordinates.
        
        The velocity consists of:
        - The vehicle's linear velocity.
        - The additional velocity caused by yaw rotation.
        
        Args:
            wheel_position_x: 
                Wheel's position relative to the vehicle centre, positive towards the front.

            wheel_position_y: 
                Wheel position relative to the vehicle centre, positiove towards the right.

        Returns:
            A tuple containing the wheel's X and Y velocity components in m/s.
        """

        # Convert the vehicle's world-space velocity into
        # vehicle-local coordinates.
        local_velocity_x, local_velocity_y = (
            self.get_local_velocity()
        )

        # Convert yaw rate from degrees per second to radians because rotational velocity uses radians.
        yaw_rate_radians = math.radians(self.yaw_rate)

        # Rotational velocity at this wheel position.
        rotational_velocity_x = (
            -yaw_rate_radians * wheel_position_y
        )

        rotational_velocity_y = (
            yaw_rate_radians * wheel_position_x
        )

        wheel_velocity_x = (
            local_velocity_x + rotational_velocity_x
        )

        wheel_velocity_y = (
            local_velocity_y + rotational_velocity_y
        )

        return wheel_velocity_x, wheel_velocity_y


    def get_local_velocity(self) -> tuple[float, float]:
        """
        Converts the vehicle's world-space velocity into vehicle-local coordinates.

        The vehicle-local X axis points forwards and the vehicle-local Y axis points to the right.

        Returns:
            A tuple containing local X and Y velocity components in metres per second.
        """

        heading_radians = math.radians(self.heading)

        local_velocity_x = (
            self.velocity_x * math.cos(heading_radians)
            + self.velocity_y * math.sin(heading_radians)
        )

        local_velocity_y = (
            -self.velocity_x * math.sin(heading_radians)
            + self.velocity_y * math.cos(heading_radians)
        )

        return local_velocity_x, local_velocity_y


    def update(self, dt: float) -> None:

        self.last_moment_z = self.moment_z

        self.acceleration_x = self.force_x / self.mass
        self.acceleration_y = self.force_y / self.mass

        # Calculate rotational acceleration from the yaw moment.
        # Moment / inertia produces radians per second squared,
        # so convert the result to degrees per second squared
        # because yaw rate and heading are stores in degrees.
        self.yaw_acceleration = math.degrees( self.moment_z / self.yaw_inertia)

        self.yaw_rate += self.yaw_acceleration * dt
        self.heading += self.yaw_rate * dt

        self.velocity_x += self.acceleration_x * dt
        self.velocity_y += self.acceleration_y * dt

        # Euler integration, we may improve this later.
        self.position_x += self.velocity_x * dt
        self.position_y += self.velocity_y * dt

        self.force_x = 0.0
        self.force_y = 0.0

        self.moment_z = 0.0