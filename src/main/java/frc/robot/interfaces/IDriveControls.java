package frc.robot.interfaces;

public interface IDriveControls {
    double GetXDrivePct();
    double GetYDrivePct();
    double GetTurnPct();
    //double GetAxis1Pct(); //Might change if design changes.
    //double GetAxis2Pct();
    //boolean IntakeWheelsRequested();
    //boolean OuttakeWheelsRequested();
    //boolean CubeGrabCloseRequested(); 
    //boolean CubeGrabOpenRequested();
    boolean BoostTriggerRequested();
    boolean PrecisionTriggerRequested();
    boolean IsFieldOrientedResetRequested();
}
