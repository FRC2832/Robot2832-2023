package frc.robot.interfaces; 

public interface IArmControl {
    void setShoulderMotorVolts(double volts);

    void setElbowMotorVolts(double volts);

    double getElbowAngle();

    double getShoulderAngle();

    void setElbowAngle(double angleDeg);
    void setShoulderAngle(double angleDeg);

    void checkBrake();

    void updateInputs();

    double getFeedForward(double offset);
}
