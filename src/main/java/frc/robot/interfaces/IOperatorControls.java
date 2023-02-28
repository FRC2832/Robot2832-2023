package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IOperatorControls {
    double GetArmShoulderPct();
    double GetArmElbowPct();
    double GetArmKinXCommand();
    double GetArmKinZCommand();
    Trigger ShoulderPosRequested();
    Trigger ShoulderNegRequested();
    Trigger ElbowPosRequested();
    Trigger ElbowNegRequested();
    JoystickButton ArmToPickupGroundCube(); //possibly need one for each game piece type
    JoystickButton ArmToPickupGroundCone(); //possibly need one for each game piece type
    JoystickButton ArmToPickupTail();
    JoystickButton ArmToPickupHuman();
    JoystickButton ArmToSecureLocation();
    JoystickButton ArmToScoreLow();
    JoystickButton ArmToScoreMiddle();
    JoystickButton ArmToScoreMiddleFront();
    JoystickButton ArmToScoreTop();
    JoystickButton ArmToTransitionPoint();
    Trigger GrabberUpRequested();
    Trigger GrabberDownRequested();
    Trigger IntakeSuckRequested();
    Trigger IntakeSpitRequested();
    Trigger ChangePieceMode();
}
