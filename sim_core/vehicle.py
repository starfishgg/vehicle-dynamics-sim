"""
vehicle.py

Contains the Vehicle class, which combines all vehicle
systems into a complete simulated vehicle.
"""

import math

from sim_core.engine import Engine
from sim_core.drivetrain import Drivetrain
from sim_core.wheel import Wheel
from sim_core.physics_state import PhysicsState
from sim_core.driver_input import DriverInput
from sim_core.math_utils import rotate_local_vector
from sim_core.telemetry import Telemetry
from sim_core.settings import FRONT_LEFT, FRONT_RIGHT




class Vehicle:

    def __init__(
        self,
        driver_input: DriverInput,
        engine: Engine,
        drivetrain: Drivetrain,
        wheels: list[Wheel],
        physics: PhysicsState,
        maximum_steering_angle: float = 30.0
    ):
        self.driver_input = driver_input
        self.engine = engine
        self.drivetrain = drivetrain
        self.wheels = wheels
        self.physics = physics

        # Maximum physical steering angle of the front wheels.
        # DriverInput.steering is normalised from -1.0 to +1.0.
        self.maximum_steering_angle = maximum_steering_angle


    def update(
        self, 
        driver_input: DriverInput,
        dt: float
    ) -> None:

        # Store current driver input for telemetry
        self.driver_input = driver_input

        # Convert normalised driver steering into a physical
        # front-wheel steering angle.
        #
        # Driver input:
        # -1.0 = full left
        #  0.0 = straight ahead
        # +1.0 = full right
        #
        # Physical wheel angle:
        # -30° = full left
        #   0° = straight ahead
        # +30° = full right

        steering_angle = (
            self.driver_input.steering * self.maximum_steering_angle
        )

        # Apply steering only to the front wheels
        for index, wheel in enumerate(self.wheels):
            if index in [FRONT_LEFT, FRONT_RIGHT]:
                wheel.steering_angle = steering_angle

        # 1. Engine produces torque
        engine_torque = self.engine.calculate_torque(self.driver_input.throttle)

        # 2. Drivetrain sends torque to driven wheels
        self.drivetrain.apply_torque(engine_torque, self.wheels)

        # 3. Update wheel rotation
        for index, wheel in enumerate(self.wheels):

            if index in self.drivetrain.driven_wheels_indices:
                wheel.update_rotation(
                    tyre_force=wheel.last_longitudinal_force,
                    dt=dt
                )
            else:
                local_velocity_x, _ = (
                    self.physics.get_local_velocity()
                )

                wheel.update_free_rolling(
                    local_velocity_x
                )

        # 4. Calculate tyre forces
        #
        # Wheel and tyre forces are calculates relative to the car:
        # X = forwards
        # Y = sideways
        #
        # We keep them in local coordinates while processing the 
        # individual wheels. The combined force is converted into
        # world coordinates after all wheel forces have been added.

        total_local_force_x = 0.0
        total_local_force_y = 0.0

        for wheel in self.wheels:

            # Calculate the velocity at this specific wheel
            # in vehicle-lcal coordinates.
            wheel_velocity_x, wheel_velocity_y = (
                self.physics.get_wheel_velocity(
                    wheel.position_x,
                    wheel.position_y
                )
            )

            slip_angle = wheel.calculate_slip_angle(
                wheel_velocity_x,
                wheel_velocity_y
            )

            wheel_speed = math.sqrt(
                wheel_velocity_x ** 2
                + wheel_velocity_y ** 2
            )

            lateral_force = wheel.tyre.calculate_lateral_force(slip_angle, wheel_speed)

            # *** TEMP TEST ***
            #print(
            #    f"{wheel.name}: "
            #    f"slip angle={slip_angle:.2f}°, "
            #    f"lateral force={lateral_force:.1f} N"
            #)


            lateral_direction_x, lateral_direction_y = (
                wheel.get_lateral_force_direction()
            )

            lateral_force_x = (
                lateral_force * lateral_direction_x
            )

            lateral_force_y = (
                lateral_force * lateral_direction_y
            )

            longitudinal_force_x, longitudinal_force_y = (
                wheel.calculate_force(wheel_velocity_x, wheel_velocity_y)
            )

            wheel_force_x = (
                longitudinal_force_x + lateral_force_x
            )

            wheel_force_y = (
                longitudinal_force_y + lateral_force_y
            )

            total_local_force_x += wheel_force_x
            total_local_force_y += wheel_force_y

            # The yaw moment is calculates from the force in the
            # vehicle's local coordinate system.
            self.physics.add_wheel_moment(
                wheel.position_x,
                wheel.position_y,
                wheel_force_x,
                wheel_force_y
            )

        # Convert the total vehicle force from local coordinates
        # into world coordinates using the vehicle's current heading.
        world_force_x, world_force_y = rotate_local_vector(
            total_local_force_x,
            total_local_force_y,
            self.physics.heading
        )

        self.physics.add_force(
            world_force_x,
            world_force_y
        )        


        # 5. Update engine RPM
        engine_rpm = self.drivetrain.calculate_engine_rpm(self.wheels)
        self.engine.set_rpm(engine_rpm)

        # 6. Move vehicle.
        self.physics.update(dt)

        # 7. Clear drive torque
        for wheel in self.wheels:
            wheel.clear_drive_torque()


    def get_telemetry(self) -> Telemetry:

        return Telemetry(
            speed_kmh=self.physics.speed * 3.6, # same as /1000 * 60 * 60
            position_x=self.physics.position_x,
            position_y=self.physics.position_y,
            acceleration=self.physics.acceleration,
            engine_rpm=self.engine.rpm,
            engine_torque=self.engine.current_torque,
            gear=self.drivetrain.gearbox.current_gear,
            throttle=self.driver_input.throttle,
            steering=self.driver_input.steering,
            tyre_forces=[
                wheel.last_longitudinal_force
                for wheel in self.wheels
            ]
        )

    def print_telemetry(self) -> None:
        current_telemetry = self.get_telemetry()

        print(f"Speed: {current_telemetry.speed_kmh:.2f} KPH")
        print(f"X: {current_telemetry.position_x:.2f}")
        print(f"Y: {current_telemetry.position_y:.2f}")
        print(f"Acceleration: {current_telemetry.acceleration:.2f} m/s²")
        print(f"Heading: {self.physics.heading:.2f}°")
        print(f"Yaw rate: {self.physics.yaw_rate:.2f}°/s")
        print(f"Yaw moment: {self.physics.last_moment_z:.2f} Nm")
        print(f"RPM: {current_telemetry.engine_rpm}")
        print(f"Gear: {current_telemetry.gear}")
        print(f"Throttle: {current_telemetry.throttle}")
        print(f"Steering: {current_telemetry.steering}")

        for index, wheel in enumerate(self.wheels):
            print(
                f"Wheel {index}: "
                f"{wheel.name} steering angle: "
                f"{wheel.steering_angle:.2f}° "
                f"\tLast Longi Force: {wheel.last_longitudinal_force:.1f} N"
            )

