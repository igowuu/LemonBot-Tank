from drivetrain import DriveTrain
import magicbot

class LemonBot(magicbot.MagicRobot):
    def createObjects(self) -> None:
        self.drivetrain = DriveTrain()

    def teleopPeriodic(self) -> None:
        self.drivetrain.driveWithJoystick()
        self.drivetrain.updateOdometry()