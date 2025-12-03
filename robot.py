from lib import LemonComponentV2, FiniteStateMachine
from wpilib import TimedRobot, XboxController
from phoenix6.hardware import TalonFX


class MyRobot(TimedRobot):
    def _set_voltage(self, voltage: float):
        self.left.setVoltage(voltage)
        self.right.setVoltage(voltage)

    def robotInit(self):
        self.fsm = FiniteStateMachine(False)
        self.xbox = XboxController(0)
        self.left = TalonFX(1)
        self.right = TalonFX(0)

        self.fsm.construct_state(
            "forward",
            lambda: self.xbox.getLeftY() > 0.3,
            lambda: self._set_voltage(0.5),
            lambda: print("forward")
        )

        self.fsm.construct_state(
            "backward",
            lambda: self.xbox.getLeftY() < -0.3,
            lambda: self._set_voltage(-0.5),
            lambda: print("backward")
        )

        self.fsm.construct_state(
            "stopped",
            lambda: abs(self.xbox.getLeftY()) < 0.3,
            lambda: self._set_voltage(0),
            lambda: print("stopped")
        )
    def teleopPeriodic(self):
      self.fsm.execute()
