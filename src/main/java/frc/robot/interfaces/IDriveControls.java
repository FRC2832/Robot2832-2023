package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public interface IDriveControls {
    double GetXDrivePct();
    double GetYDrivePct();
    double GetTurnPct();
    double GetArmShoulderPct();
    double GetArmElbowPct();
    double GetArmKinXCommand();
    double GetArmKinZCommand();
    boolean IntakeConeRequested();
    boolean OuttakeConeRequested();
    JoystickButton CubeGrabCloseRequested(); 
    JoystickButton CubeGrabOpenRequested();
    boolean BoostTriggerRequested();
    boolean PrecisionTriggerRequested();
    boolean IsFieldOrientedResetRequested();
}
