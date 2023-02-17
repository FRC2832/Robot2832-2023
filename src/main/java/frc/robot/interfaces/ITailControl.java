package frc.robot.interfaces;


public interface ITailControl {
    void setTailAngle(double angleDeg);
    void setTailVoltage(double volts);
    

    double getTailAngle();
    double getDistSensor();

    void updateInputs();
}
