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
            Translation2d(0.381, 0.381),    # front left
            Translation2d(0.381, -0.381),   # front right
            Translation2d(-0.381, 0.381),   # back left
            Translation2d(-0.381, -0.381)   # back right
        ]

        self.modules = [
            SwerveModule(1, 5, 9),  # front left
            SwerveModule(2, 6, 10),   # front right
            SwerveModule(3, 7, 11),  # back left
            SwerveModule(4, 8, 12)   # back right
        ]

        self.gyro = Pigeon2(13, canbus="can0")
        self.gyro.reset()

        self.kinematics = SwerveDrive4Kinematics(*self.module_locations)
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
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

        module_states = self.kinematics.toSwerveModuleStates(ChassisSpeeds(xSpeed, ySpeed, rot))
        for module, state in zip(self.modules, module_states):
            module.setDesiredState(state)

    def _create_states(self) -> None:
        def apply_deadband(value: float) -> float:
            if abs(value) < DEADBAND:
                return 0.0
            return value

        self.fsm.construct_state(
            "forward",
            lambda: apply_deadband(self.lstick.getY()) < -DEADBAND,
            lambda: self.drive(
                0, -apply_deadband(self.lstick.getY()) * MAX_SPEED_MPS, 0
            ),
            lambda: print("MOVE: forward")
        )

        self.fsm.construct_state(
            "backward",
            lambda: apply_deadband(self.lstick.getY()) > DEADBAND,
            lambda: self.drive(
                0, -apply_deadband(self.lstick.getY()) * MAX_SPEED_MPS, 0
            ),
            lambda: print("MOVE: backward")
        )

        self.fsm.construct_state(
            "left",
            lambda: apply_deadband(self.lstick.getX()) > DEADBAND,
            lambda: self.drive(
                -apply_deadband(-self.lstick.getX()) * MAX_SPEED_MPS, 0, 0
            ),
            lambda: print("MOVE: left")
        )

        self.fsm.construct_state(
            "right",
            lambda: apply_deadband(self.lstick.getX()) < -DEADBAND,
            lambda: self.drive(
                -apply_deadband(-self.lstick.getX()) * MAX_SPEED_MPS, 0, 0
            ),
            lambda: print("MOVE: right")
        )

        self.fsm.construct_state(
            "left_turn",
            lambda: apply_deadband(self.rstick.getX()) < -DEADBAND,
            lambda: self.drive(
                0, 0, -apply_deadband(self.rstick.getX()) * MAX_OMEGA_PS
            ),
            lambda: print("TURN: left")
        )

        self.fsm.construct_state(
            "right_turn",
            lambda: apply_deadband(self.rstick.getX()) > DEADBAND,
            lambda: self.drive(
                0, 0, -apply_deadband(self.rstick.getX()) * MAX_OMEGA_PS
            ),
            lambda: print("TURN: right")
        )

        self.fsm.construct_state(
            "stopped",
            lambda: (
                abs(self.lstick.getY()) <= DEADBAND and
                abs(self.lstick.getX()) <= DEADBAND and
                abs(self.rstick.getX()) <= DEADBAND
            ),
            lambda: self.drive(0, 0, 0),
            lambda: print("Stopped")
        )




    def updateOdometry(self) -> None:
        """Updates the position of the robot on the field."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            tuple(module.getPosition() for module in self.modules)
        )