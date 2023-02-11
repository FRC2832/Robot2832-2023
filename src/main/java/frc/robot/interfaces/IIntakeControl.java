package frc.robot.interfaces;

public interface IIntakeControl {
    
    void setIntakeAngle(double angleDeg);

    void setIntakeMotorVolts(double volts);

    double getIntakeAngle();

    void updateInputs();

}
