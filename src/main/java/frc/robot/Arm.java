package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.interfaces.IArmControl;

public class Arm implements Subsystem{
    private IArmControl hardware;
    
    public Arm(IArmControl hardware) {
        this.hardware = hardware;
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
        SmartDashboard.putNumber("Shoulder Angle", getShoulderAngle());
        SmartDashboard.putNumber("Elbow Angle", getElbowAngle());
    }

    public void setShoulderAngle(double angleDeg) {
        hardware.setShoulderAngle(angleDeg);
    }

    public void setElbowAngle(double angleDeg) {
        hardware.setElbowAngle(angleDeg);
    }

    
    public double getShoulderAngle() {
        return hardware.getShoulderAngle();
    }
    
    public double getElbowAngle() {
        return hardware.getElbowAngle();
    }

    public void setShoulderMotorVolts(double volts) {
        hardware.setShoulderMotorVolts(volts);
    }

    public void setElbowMotorVolts(double volts) {
        hardware.setElbowMotorVolts(volts);
    }
}
