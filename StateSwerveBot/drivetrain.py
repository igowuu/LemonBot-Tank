from swervemodule import SwerveModule
from wpimath.geometry import Translation2d
from phoenix6.hardware import Pigeon2
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, ChassisSpeeds
import wpilib
import wpimath.filter

MAX_SPEED_MPS = 4.7
MAX_OMEGA_PS = 6 # rads / sec

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
        self.omegaLimiter = wpimath.filter.SlewRateLimiter(3)

        self.xSpeed = 0
        self.ySpeed = 0
        self.omega = 0

    def drive(self, xSpeed: float, ySpeed: float, omega: float) -> None:
        """Drives the bot and updates current velocity and rotation fields."""
        self.xSpeed = xSpeed
        self.ySpeed = ySpeed
        self.omega = omega

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, omega, self.gyro.getRotation2d()
        )

        module_states = self.kinematics.toSwerveModuleStates(chassisSpeeds)
        self.kinematics.desaturateWheelSpeeds(module_states, MAX_SPEED_MPS)
        
        for module, state in zip(self.modules, module_states):
            module.setDesiredState(state)

    def updateOdometry(self) -> None:
        """Updates the position of the robot on the field."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            tuple(module.getPosition() for module in self.modules)
        )

    def driveWithJoystick(self) -> None:
        self.xSpeed = (
            self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.lstick.getX(), 0.02)
            ) * MAX_SPEED_MPS
        )

        self.ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.lstick.getY(), 0.02)
            ) * MAX_SPEED_MPS
        )

        self.omega = (
            -self.omegaLimiter.calculate(
                wpimath.applyDeadband(self.rstick.getX(), 0.02)
            ) * MAX_OMEGA_PS
        )
        
        self.drive(self.xSpeed, self.ySpeed, self.omega)