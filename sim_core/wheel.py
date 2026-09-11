"""
wheel.py

Contains the Wheel class, whichmodels a vehicle wheel and
converts drivetrain torque into tyre forces.
"""

import math

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
        tyre: Tyre | None = None,
        position_x: float = 0.0,
        position_y: float = 0.0,
        steering_angle: float = 0.0
    ):
        self.radius = radius # in metres
        self.tyre = tyre
        self.name = name

        # Wheel position relative to vehicles centre of mass
        # X: positive is towards the front of the vehicle
        # Y: positive is towards the right side of the vehicle
        self.position_x: float = position_x
        self.position_y: float = position_y

        # Steering angle of the wheel in degrees.
        # 0 degrees means the wheel points straight ahead.
        self.steering_angle: float = steering_angle

        # Torque applied by drivetrain
        self.drive_torque = 0.0

        # Wheel rotational speed in radians per second.
        self.angular_velocity = 0.0 # in radians/sec.

        # Approximate rotational inertia of the wheel and tyre assembly.
        self.rotational_inertia: float = 2.0 # kg m²

        # Force calculated during the current timestep.
        self.last_longitudinal_force = 0.0

        


    def update_rotation(
            self,
            tyre_force: float,
            dt: float
    ) -> None:
        """
        Updates wheel angular velocity from applied torque
        and tyre reaction torque.
        """

        if self.rotational_inertia <= 0:
            raise ValueError("Rotational inertia must be positive")

        tyre_torque = tyre_force * self.radius

        net_torque = self.drive_torque - tyre_torque

        angular_acceleration = net_torque / self.rotational_inertia

        self.angular_velocity += angular_acceleration * dt


    def calculate_slip_ratio(
            self,
            wheel_velocity_x: float,
            wheel_velocity_y: float
    ) -> float:
        """
        Calculates longitudinal slip ratio.

        The wheel's velocity is projected onto the direction the wheel is ponting. This means a steered wheel uses its actual rolling velocity rather than simply using the vehicle's forward velocity.

        Args:
            wheel_velocity_x:
                Wheel velocity in vehicle-local X, in m/s.

            wheel_velocity_y:
                Wheel velocity in vehicle-local Y, in m/s.

        Returns:
            Longitudinal slip ratio.
        """ 

        direction_x, direction_y = self.get_force_direction()

        rolling_velocity = (
            wheel_velocity_x * direction_x
            + wheel_velocity_y * direction_y
        )

        wheel_speed = self.angular_velocity * self.radius

        reference_speed = max(abs(rolling_velocity), 0.1)

        return (wheel_speed - rolling_velocity) / reference_speed



    def calculate_force(
            self,
            wheel_velocity_x: float,
            wheel_velocity_y: float
    ) -> tuple[float, float]:
        """
        Calculates the longitudinal tyre force and resolves it into vehicle X and Y components based
        on steering angle.

        Args:
            wheel_velocity_x:
                Wheel velocity in the vehicle's forward direction, in m/s.

        Returns:
            A tuple containing the X and Y components of the tyre force in Newtons.
        """

        if self.tyre is None:
            raise ValueError("Wheel has no tyre attached")

        slip_ratio = self.calculate_slip_ratio(
            wheel_velocity_x,
            wheel_velocity_y
        )

        self.last_longitudinal_force = (
            self.tyre.calculate_longitudinal_force(
                slip_ratio
            )
        )

        direction_x, direction_y = self.get_force_direction()

        force_x = (
            self.last_longitudinal_force * direction_x
        )

        force_y = (
            self.last_longitudinal_force * direction_y
        )

        return (force_x, force_y)


    def apply_drive_torque(self, torque: float) -> None:
        self.drive_torque = torque


    def clear_drive_torque(self) -> None:
        self.drive_torque = 0.0


    def update_free_rolling(
            self,
            vehicle_speed: float,
    ) -> None:
        """
        Sets the wheel rotation to match the vehicle's
        longitudinal road speed.
        
        A free-rolling wheel has no drive or brake torque,
        so it should rotate at the speed required to roll
        without longitudinal slip.
        """

        self.angular_velocity = vehicle_speed / self.radius


    def get_force_direction(self) -> tuple[float, float]:
        """
        Calculates the direction the wheel is pointing.
        
        Returns:
            A tuple containing the X and Y components of the wheel's forward direction.
            
        A steering angle of 0 degrees points straight ahead along the vehicle's positive X axis.
        """

        steering_angle_radians = math.radians(
            self.steering_angle
        )

        direction_x = math.cos(steering_angle_radians)
        direction_y = math.sin(steering_angle_radians)

        return (direction_x, direction_y)


    def get_lateral_force_direction(self) -> tuple[float, float]:
        """
        Calculates the sideways direction of the wheel.

        The lateral direction is perpendicular to the wheel's forward direction.

        Returns:
            A tuple containing the X and Y components of the wheel's lateral direction.
        """

        steering_angle_radians = math.radians(
            self.steering_angle
        )

        lateral_direction_x = -math.sin(
            steering_angle_radians
        )

        lateral_direction_y = math.cos(
            steering_angle_radians
        )

        return lateral_direction_x, lateral_direction_y


    def calculate_slip_angle(
            self,
            wheel_velocity_x: float,
            wheel_velocity_y: float
    ) -> float:
        """
        Calculates the angle between the wheel's pointing direction and its actual direction of travel.

        Args:
            wheel_velocity_x:
                Wheel's velocity in the vehicle's forward direction in m/s.

            wheel_velocity_y:
                Wheel's velocity sideways in m/s.

        Returns:
            Slip angle in degrees.
        """

        # Determines the drection the weel is actually travelling.
        travel_angle = math.atan2(
            wheel_velocity_y,
            wheel_velocity_x
        )

        # Convert the wheel's steering angle to radians.
        steering_angle_radians = math.radians(
            self.steering_angle
        )

        # Slip angle is the dirrerence between where the 
        # wheel is pointing and where it is travelling.
        slip_angle = steering_angle_radians - travel_angle

        # Convert slip angle to degrees.
        return math.degrees(slip_angle)
