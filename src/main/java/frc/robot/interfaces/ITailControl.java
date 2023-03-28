package frc.robot.interfaces;


public interface ITailControl {
    void setTailVoltage(double volts);

    double getTailAngle();
    double getDistSensor();

    void updateInputs();
}
