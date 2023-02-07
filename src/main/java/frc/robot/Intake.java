package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.interfaces.IIntakeControl;

public class Intake implements Subsystem { 
    private IIntakeControl hardware;
    private double intakeAng;
    
    public Intake(IIntakeControl hardware){
        this.hardware = hardware;
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
        SmartDashboard.putNumber("Intake Angle", getIntakeAngle());
    }

    void setIntakeAngle(double angleDeg){
        hardware.setIntakeAngle(angleDeg);
    }

    void setIntakeMotorVolts(double volts){
        hardware.setIntakeMotorVolts(volts);
    }

    double getIntakeAngle(){
        intakeAng = hardware.getIntakeAngle();
        return intakeAng;
    }
}
