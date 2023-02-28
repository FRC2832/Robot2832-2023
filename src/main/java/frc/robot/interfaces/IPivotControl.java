package frc.robot.interfaces;

public interface IPivotControl {
    
    void setPivotAngle(double angleDeg);

    void setPivotMotorVolts(double volts);

    double getPivotAngle();
    void resetRotations();

    void updateInputs();

}
