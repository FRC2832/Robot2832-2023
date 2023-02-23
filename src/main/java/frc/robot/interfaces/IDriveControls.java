package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Arm;
import frc.robot.GrabberIntake;
import frc.robot.Intake;

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
    void initializeButtons(Arm arm, Intake intake, GrabberIntake grabber);
}
