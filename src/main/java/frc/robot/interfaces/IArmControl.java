package frc.robot.interfaces; 

public interface IArmControl {
    void setShoulderAngle(double angleDeg);

    void setElbowAngle(double angleDeg);
    
    void setShoulderMotorVolts(double volts);

    void setElbowMotorVolts(double volts);

    double getElbowAngle();

    double getShoulderAngle();

    void checkBrake();

    void updateInputs();

}
