package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.IPivotControl;

public class Pivot extends SubsystemBase { 
    private IPivotControl hardware;
    private Arm arm;
    
    public Pivot(IPivotControl hardware, Arm arm){
        super();
        this.hardware = hardware;
        this.arm = arm;
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
        SmartDashboard.putNumber("Optimal Angle", optimalPivotAngle());
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

    public double optimalPivotAngle(){
        double value;
        if(Robot.getGamePieceMode() == Robot.CONE_MODE) { 
            value =  arm.getElbowAngle() + arm.getShoulderAngle();
        }
        else { 
            value =  90 + arm.getElbowAngle() + arm.getShoulderAngle();
        }

        //flip offset based on arm position
        double offset;
        if(arm.getArmXPosition() > 0) {
            offset = 35;
        } else {
            offset = -40;
        }
        return value - offset;  //optimum angle to "snowblower" the piece in
    }
}
