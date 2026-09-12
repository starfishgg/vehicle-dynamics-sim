"""
simulation.py

Runs a basic vehicle simulation.
"""

from sim_core.vehicle import Vehicle
from sim_core.driver_input import DriverInput
from sim_core.tyre import Tyre
from sim_core.wheel import Wheel
from sim_core.engine import Engine
from sim_core.drivetrain import Drivetrain
from sim_core.physics_state import PhysicsState
from sim_core.gearbox import Gearbox
from sim_core.differential import Differential
from sim_core.settings import *




def create_test_vehicle():
    controller = DriverInput(
        throttle=1.0,
        steering=0.33
    )

    tyres = [
        Tyre(),
        Tyre(),
        Tyre(),
        Tyre()
    ]

    wheels = [
        Wheel(
            name="Front Left", 
            tyre=tyres[FRONT_LEFT],
            position_x=1.4,
            position_y=-0.8,
            steering_angle=0.0
        ),
        Wheel(
            name="Front Right", 
            tyre=tyres[FRONT_RIGHT],
            position_x=1.4,
            position_y=0.8,
            steering_angle=0.0
        ),
        Wheel(
            name="Rear Left", 
            tyre=tyres[REAR_LEFT],
            position_x=-1.4,
            position_y=-0.8,
            steering_angle=0.0
        ),
        Wheel(
            name="Rear Right", 
            tyre=tyres[REAR_RIGHT],
            position_x=-1.4,
            position_y=0.8,
            steering_angle=0.0
        )
    ]



    return Vehicle(
        driver_input=controller,
        engine=Engine(),
        drivetrain=Drivetrain(
            gearbox=Gearbox(),
            driven_wheels_indices=[REAR_LEFT, REAR_RIGHT],
            differential=Differential()
        ),
        wheels=wheels,
        physics=PhysicsState()
    )



def run():

    dt = 0.01 # 100 Hz simulation



    car = create_test_vehicle()


    for step in range(1000):

        car.update(car.driver_input, dt)
        
        if step % 100 == 0:

            print(
                f"Time: {step * dt:.2f}s "
                f"Speed: {car.physics.speed:.2f} m/s "
                f"Position: {car.physics.position_x:.2f}m"
            )
            car.print_telemetry()


if __name__ == "__main__":
    run()
