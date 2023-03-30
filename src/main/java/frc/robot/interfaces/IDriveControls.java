package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public interface IDriveControls {
    double GetXDrivePct();
    double GetYDrivePct();
    double GetTurnPct();
    double GetBoostTriggerRequest();
    double GetPrecisionTriggerRequest();
    boolean IsFieldOrientedResetRequested();
    JoystickButton TailUpRequested();
    JoystickButton TailDownRequested();
    JoystickButton TailStowRequested();
    JoystickButton TailHumanRequested();
}
