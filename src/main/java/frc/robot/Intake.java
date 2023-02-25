package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.IIntakeControl;

public class Intake extends SubsystemBase { 
    private IIntakeControl hardware;
    private Arm arm;
    
    public Intake(IIntakeControl hardware, Arm arm){
        super();
        this.hardware = hardware;
        this.arm = arm;
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
        SmartDashboard.putNumber("Optimal Angle", optimalIntakeAngle());
    }

    public void setPivotAngle(double angleDeg){
        hardware.setPivotAngle(angleDeg);
    }

    public void setPivotMotorVolts(double volts){
        hardware.setPivotMotorVolts(volts);
    }

    public double getPivotAngle(){
        return hardware.getPivotAngle();
    }

    public void resetRotations() {
        hardware.resetRotations();
    }

    public double optimalIntakeAngle(){
        double value;
        if(Robot.getGamePieceMode()) { //pieceMode true is cube
            value =  arm.getElbowAngle() + arm.getShoulderAngle();
        }
        else { //pieceMode false is cone
            value =  90 + arm.getElbowAngle() + arm.getShoulderAngle();
        }

        //flip offset based on arm position
        double offset;
        if(arm.getArmXPosition() > 0) {
            offset = 35;
        } else {
            offset = -35;
        }
        return value - offset;  //optimum angle to "snowblower" the piece in
    }
}
