from swervemodule import SwerveModule
from wpimath.geometry import Translation2d
from phoenix6.hardware import Pigeon2
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, ChassisSpeeds
import math
import wpilib
import wpimath.filter
from states import FiniteStateMachine

MAX_SPEED_MPS = 6.0
MAX_OMEGA_PS = math.pi * 3
DEADBAND = 0.02

class DriveTrain:

    def __init__(self) -> None:
        self.module_locations = [
            Translation2d(0.381, 0.381),  # front left
            Translation2d(0.381, -0.381),  # front right
            Translation2d(-0.381, 0.381),  # back left
            Translation2d(-0.381, -0.381)  # back right
        ]

        self.modules = [
            SwerveModule(1, 5, 9),  # front left
            SwerveModule(2, 6, 10),  # front right
            SwerveModule(3, 7, 11),  # back left
            SwerveModule(4, 8, 12)  # back right
        ]

        self.gyro = Pigeon2(13, canbus="can0")
        self.gyro.reset()

        self.kinematics = SwerveDrive4Kinematics(*self.module_locations)
        self.odometry = SwerveDrive4Odometry(
            self.kinematics, self.gyro.getRotation2d(),
            tuple(module.getPosition() for module in self.modules)
        )

        self.lstick = wpilib.Joystick(0)
        self.rstick = wpilib.Joystick(1)

        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        self.xSpeed = 0
        self.ySpeed = 0
        self.rot = 0

        self.fsm = FiniteStateMachine(isInTest=False)
        self._create_states()

    def drive(self, xSpeed: float, ySpeed: float, rot: float) -> None:
        """Drives the bot and updates current velocity and rotation fields."""
        self.xSpeed = xSpeed
        self.ySpeed = ySpeed
        self.rot = rot

        chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, self.gyro.getRotation2d()
        )

        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)

        for module, state in zip(self.modules, module_states):
            module.setDesiredState(state)

    def _create_states(self) -> None:
        """Constructs FSM states."""
        self.fsm.construct_state(
            "teleop_drive",
            lambda: True,  # Always true, add logic to ensure teleop mode
            lambda: self.drive(
                self.lstick.getX() * MAX_SPEED_MPS, 
                -self.lstick.getY() * MAX_SPEED_MPS, 
                -self.rstick.getX() * MAX_OMEGA_PS
            ),
            lambda: print("MODE: Teleop")
        )

    def updateOdometry(self) -> None:
        """Updates the position of the robot on the field."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            tuple(module.getPosition() for module in self.modules)
        )
