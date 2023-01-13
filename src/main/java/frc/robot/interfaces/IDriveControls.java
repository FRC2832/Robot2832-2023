package frc.robot.interfaces;

public interface IDriveControls {
    double GetXDrivePct();
    double GetYDrivePct();
    double GetTurnPct();
    boolean IsFieldOrientedResetRequested();
}
