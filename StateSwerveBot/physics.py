import math
import typing

from pyfrc.physics.core import PhysicsInterface
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

if typing.TYPE_CHECKING:
    from robot import LemonBot

WHEEL_RADIUS_M = 0.05
GEAR_RATIO = 6.75

class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "LemonBot"):
        self.physics_controller = physics_controller
        self.robot = robot
        self.drivetrain = robot.drivetrain

        self.modules = self.drivetrain.modules

        self.gyro_sim = self.drivetrain.gyro.sim_state 

        self.pose = Pose2d()

        self.wheel_rotor_positions = {
            'frontLeft': 0.0,
            'frontRight': 0.0,
            'backLeft': 0.0,
            'backRight': 0.0
        }

        self._wheel_names = ["frontLeft", "frontRight", "backLeft", "backRight"]

        self._meters_per_motor_rev = (
            2 * math.pi * WHEEL_RADIUS_M
        ) / GEAR_RATIO

        self.drivetrain.gyro.reset()

    def update_sim(self, _: float, tm_diff: float) -> None:
        vx = self.drivetrain.xSpeed
        vy = self.drivetrain.ySpeed
        omega = self.drivetrain.omega
        
        chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega, self.drivetrain.gyro.getRotation2d()
        )

        self.pose = self.physics_controller.drive(chassis_speeds, tm_diff)

        states = self.drivetrain.kinematics.toSwerveModuleStates(chassis_speeds)

        for name, module, state in zip(self._wheel_names, self.modules, states):
            wheel_speed = state.speed

            delta_revs = (wheel_speed * tm_diff) / self._meters_per_motor_rev
            self.wheel_rotor_positions[name] += delta_revs

            steer_angle = state.angle.radians()
            revs = steer_angle / (2 * math.pi)

            module.steer_motor.sim_state.set_raw_rotor_position(revs)
            module.cancoder.sim_state.set_raw_position(revs)

            module.drive_motor.sim_state.set_raw_rotor_position(
                self.wheel_rotor_positions[name]
            )
        
        yaw_deg = self.pose.rotation().degrees()
        self.gyro_sim.set_raw_yaw(yaw_deg)

        self.drivetrain.updateOdometry()