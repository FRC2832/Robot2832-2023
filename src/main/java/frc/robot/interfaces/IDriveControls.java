package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Arm;
import frc.robot.GrabberIntake;
import frc.robot.Intake;

public interface IDriveControls {
    double GetXDrivePct();
    double GetYDrivePct();
    double GetTurnPct();
    double GetArmShoulderPct();
    double GetArmElbowPct();
    double GetArmKinXCommand();
    double GetArmKinZCommand();
    double GetPercentRightTriggerAxis();
    double GetPercentLeftTriggerAxis();
    boolean BoostTriggerRequested();
    boolean PrecisionTriggerRequested();
    boolean IsFieldOrientedResetRequested();
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
    JoystickButton TailUpRequested();
    JoystickButton TailDownRequested();
    JoystickButton GrabberUpRequested();
    JoystickButton GrabberDownRequested();
    double GetGrabberPct();
    JoystickButton GrabberSuckRequested();
    JoystickButton GrabberSpitRequested();
    void initializeButtons(Arm arm, Intake intake, GrabberIntake grabber);
    JoystickButton ChangePieceMode();
}
