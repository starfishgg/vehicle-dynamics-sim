"""
vehicle.py

Contains the Vehicle class, which combines all vehicle
systems into a complete simulated vehicle.
"""

from sim_core.engine import Engine
from sim_core.drivetrain import Drivetrain
from sim_core.wheel import Wheel
from sim_core.physics_state import PhysicsState
from sim_core.driver_input import DriverInput
from sim_core.math_utils import rotate_local_vector
from sim_core.telemetry import Telemetry




class Vehicle:

    def __init__(
        self,
        driver_input: DriverInput,
        engine: Engine,
        drivetrain: Drivetrain,
        wheels: list[Wheel],
        physics: PhysicsState

    ):
        self.driver_input = driver_input
        self.engine = engine
        self.drivetrain = drivetrain
        self.wheels = wheels
        self.physics = physics


    def update(self, dt: float) -> None:

        # 1. Engine produces torque
        engine_torque = self.engine.calculate_torque(self.driver_input.throttle)

        # 2. Drivetrain sends torque to driven wheels
        self.drivetrain.apply_torque(engine_torque, self.wheels)

        # 3. Wheels produce forces
        for wheel in self.wheels:

            wheel_force = wheel.calculate_force()

            force_x, force_y = rotate_local_vector(
                wheel_force,
                self.physics.heading
            )

            self.physics.add_force(force_x, force_y)

        # 4. Physics integration
        self.physics.update(dt)

        # 5. Clear temporary forces
        for wheel in self.wheels:
            wheel.clear_drive_torque()


    def get_telemetry(self) -> Telemetry:

        return Telemetry(
            speed_kmh=self.physics.speed/1000*60*60,
            position_x=self.physics.position_x,
            position_y=self.physics.position_y,
            acceleration=self.physics.acceleration,
            engine_rpm=self.engine.rpm,
            engine_torque=self.engine.calculate_torque(self.driver_input.throttle),
            gear=self.drivetrain.gearbox.current_gear,
            throttle=self.driver_input.throttle,
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
        print(f"RPM: {current_telemetry.engine_rpm}")
        print(f"Gear: {current_telemetry.gear}")
        print(f"Throttle: {current_telemetry.throttle}")

        for index, wheel in enumerate(self.wheels):
            print(
                f"Wheel {index}: "
                f"{wheel.last_longitudinal_force:.1f} N"
            )

