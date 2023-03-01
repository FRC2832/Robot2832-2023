package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public interface IDriveControls {
    double GetXDrivePct();
    double GetYDrivePct();
    double GetTurnPct();
    double GetPercentRightTriggerAxis();
    double GetPercentLeftTriggerAxis();
    boolean BoostTriggerRequested();
    boolean PrecisionTriggerRequested();
    boolean IsFieldOrientedResetRequested();
    JoystickButton TailUpRequested();
    JoystickButton TailDownRequested();
    boolean checkController();
}
