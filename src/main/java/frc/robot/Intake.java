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
        SmartDashboard.putNumber("Intake Angle", getPivotAngle());
    }

    public void setPivotAngle(double angleDeg){
        hardware.setPivotAngle(angleDeg);
    }

    public void setPivotMotorVolts(double volts){
        hardware.setPivotMotorVolts(volts);
    }

    double getPivotAngle(){
        intakeAng = hardware.getPivotAngle();
        return intakeAng;
    }
}
