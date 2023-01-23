package frc.robot.interfaces;

public interface IDriveControls {
    double GetXDrivePct();
    double GetYDrivePct();
    double GetTurnPct();
    double GetArmAxis1Pct(); //Might change if design changes.
    double GetArmAxis2Pct();
    boolean IntakeConeRequested();
    boolean OuttakeConeRequested();
    boolean CubeGrabCloseRequested(); 
    boolean CubeGrabOpenRequested();
    boolean BoostTriggerRequested();
    boolean PrecisionTriggerRequested();
    boolean IsFieldOrientedResetRequested();
}
