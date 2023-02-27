package frc.robot.interfaces; 

public interface IArmControl {
    void setShoulderMotorVolts(double volts);

    void setElbowMotorVolts(double volts);

    double getElbowAngle();

    double getShoulderAngle();

    void checkBrake();

    void updateInputs();

    double getFeedForward(double offset);
}
