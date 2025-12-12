import math

from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from phoenix6.hardware import TalonFX, CANcoder

DRIVE_KP = 0.0
DRIVE_KI = 0.0
DRIVE_KD = 0.0
DRIVE_KS = 0.17
DRIVE_KV = 0.104
DRIVE_KA = 0.01

STEER_KP = 3.0
STEER_KI = 0.0
STEER_KD = 0.0

WHEEL_DIAMETER_M = 0.1016
WHEEL_RADIUS_M = 0.0508
GEAR_RATIO = 6.75
MAX_SPEED_MPS = 4.7
MAX_VOLTAGE = 12

class SwerveModule:
    def __init__(self, drive_id: int, steer_id: int, cancoder_id: int) -> None:
        self.drive_motor = TalonFX(drive_id, canbus="can0")
        self.steer_motor = TalonFX(steer_id, canbus="can0")
        self.cancoder = CANcoder(cancoder_id, canbus="can0")

        self.drive_motor.set_position(0)

        self.steer_pid = PIDController(STEER_KP, STEER_KI, STEER_KD)
        self.steer_pid.enableContinuousInput(-math.pi, math.pi)

        self.drive_pid = PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD)
        self.drive_ff = SimpleMotorFeedforwardMeters(DRIVE_KS, DRIVE_KV, DRIVE_KA)

    def _read_cancoder_angle_radians(self) -> float:
        """ Returns the CANcoder absolute position in radians instead of rotations. """
        rotations = self.cancoder.get_absolute_position().value
        angle_rad = rotations * (2.0 * math.pi)
        return angle_rad
    
    def _applySteerControl(self, desired_angle: float):
        """ Drives the steering motor toward the target angle. """
        current_angle = self._read_cancoder_angle_radians()
        pid_voltage = self.steer_pid.calculate(current_angle, desired_angle)

        voltage = max(min(pid_voltage, MAX_VOLTAGE), -MAX_VOLTAGE)
        self.steer_motor.setVoltage(voltage)
    
    def getPosition(self) -> SwerveModulePosition:
        """ Returns the total drive position and rotation of the wheels. """
        rotor_rotations = self.drive_motor.get_position().value
        wheel_rotations = rotor_rotations / GEAR_RATIO
        drive_position_m = wheel_rotations * (2.0 * math.pi * WHEEL_RADIUS_M)

        angle_rad = self._read_cancoder_angle_radians()
        return SwerveModulePosition(drive_position_m, Rotation2d(angle_rad))

    def setDesiredState(self, desired_state: SwerveModuleState) -> None:
        """ Commands the SwerveModule to get to the provided state. """
        current_angle = Rotation2d(self._read_cancoder_angle_radians())
        desired_state.optimize(current_angle)

        desired_speed_mps = desired_state.speed # meters/sec

        ff_voltage = self.drive_ff.calculate(desired_speed_mps)

        current_rotor_rate = self.drive_motor.get_velocity().value  # rotations/sec
        current_wheel_rate_mps = (current_rotor_rate / GEAR_RATIO) * 2 * math.pi * WHEEL_RADIUS_M
        pid_voltage = self.drive_pid.calculate(current_wheel_rate_mps, desired_speed_mps)

        total_voltage = ff_voltage + pid_voltage
        total_voltage = max(min(total_voltage, MAX_VOLTAGE), -MAX_VOLTAGE)
        self.drive_motor.setVoltage(total_voltage)

        self._applySteerControl(desired_state.angle.radians())