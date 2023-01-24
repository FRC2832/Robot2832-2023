package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public interface IDriveControls {
    double GetXDrivePct();
    double GetYDrivePct();
    double GetTurnPct();
    double GetArmAxis1Pct(); //Might change if design changes.
    double GetArmAxis2Pct();
    boolean IntakeConeRequested();
    boolean OuttakeConeRequested();
    JoystickButton CubeGrabCloseRequested(); 
    JoystickButton CubeGrabOpenRequested();
    boolean BoostTriggerRequested();
    boolean PrecisionTriggerRequested();
    boolean IsFieldOrientedResetRequested();
}
