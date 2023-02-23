package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Arm;
import frc.robot.GrabberIntake;
import frc.robot.Intake;

public interface IOperatorControls {
    double GetArmShoulderPct();
    double GetArmElbowPct();
    double GetArmKinXCommand();
    double GetArmKinZCommand();
    JoystickButton ShoulderPosRequested();
    JoystickButton ShoulderNegRequested();
    JoystickButton ElbowPosRequested();
    JoystickButton ElbowNegRequested();
    JoystickButton ArmToPickupGround(); //possibly need one for each game piece type
    JoystickButton ArmToPickupTail();
    JoystickButton ArmToPickupHuman();
    JoystickButton ArmToSecureLocation();
    JoystickButton ArmToScoreLow();
    JoystickButton ArmToScoreMiddle();
    JoystickButton ArmToScoreTop();
    JoystickButton GrabberUpRequested();
    JoystickButton GrabberDownRequested();
    JoystickButton GrabberSuckRequested();
    JoystickButton GrabberSpitRequested();
    double GetGrabberPct();
    void initializeButtons(Arm arm, Intake intake, GrabberIntake grabber);
    JoystickButton ChangePieceMode();
}
