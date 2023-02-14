package frc.robot;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.interfaces.IArmControl;

public class Arm implements Subsystem{
    private IArmControl hardware;
    private double shoulderAng;
    private double elbowAng;
    
    public Arm(IArmControl hardware) {
        this.hardware = hardware;
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
        SmartDashboard.putNumber("Shoulder Angle", getShoulderAngle());
        SmartDashboard.putNumber("Elbow Angle", getElbowAngle());
        SmartDashboard.putNumber("Arm X Position", getArmXPosition());
        SmartDashboard.putNumber("Arm Z Position", getArmZPosition());
        hardware.checkBrake();
    }

    public void setShoulderAngle(double angleDeg) {
        hardware.setShoulderAngle(angleDeg);
    }

    public void setElbowAngle(double angleDeg) {
        hardware.setElbowAngle(angleDeg);
    }

    
    public double getShoulderAngle() {
        shoulderAng = hardware.getShoulderAngle();
        return shoulderAng;
    }
    
    public double getElbowAngle() {
        elbowAng = hardware.getElbowAngle();
        return elbowAng;
    }

    public void setShoulderMotorVolts(double volts) {
        hardware.setShoulderMotorVolts(volts);
    }

    public void setElbowMotorVolts(double volts) {
        hardware.setElbowMotorVolts(volts);
    }

    //this one will become not good once we figure out the equation for an arm with two segments of different lengths
    public void calcAngles(double x, double z) { //calculate the angles for each part of the arm to get to the point (x, z)
        // from https://www.youtube.com/watch?v=Q-UeYEpwXXU
        double shoulder = 0;
        double elbow = 0;
        //40-45 finds elbow angle in radians, but havent tested to see which angle is for the up reaching arm and down reaching arm
        if(x > 0) {
            elbow =  Math.acos((Constants.BICEP_LENGTH*Constants.BICEP_LENGTH + Constants.FOREARM_LENGTH*Constants.FOREARM_LENGTH - x*x - z*z)/(2*Constants.BICEP_LENGTH*Constants.FOREARM_LENGTH)) - 3.14159;
            shoulder = Math.atan(z/x) - Math.atan((Constants.FOREARM_LENGTH*Math.sin(elbow))/(Constants.BICEP_LENGTH + Constants.FOREARM_LENGTH*Math.cos(elbow)));
        }
        else if(x < 0){
            elbow = Math.acos((x*x + z*z - Constants.BICEP_LENGTH*Constants.BICEP_LENGTH - Constants.FOREARM_LENGTH*Constants.FOREARM_LENGTH)/(2*Constants.BICEP_LENGTH*Constants.FOREARM_LENGTH));
            shoulder = 3.14159 - Math.atan(z/x) - Math.atan((Constants.FOREARM_LENGTH*Math.sin(elbow))/(Constants.BICEP_LENGTH + Constants.FOREARM_LENGTH*Math.cos(elbow)));
        }
        else {
            return;
        }
        // finds shoulder angle in radians
        
        elbow = Math.toDegrees(elbow);
        shoulder = Math.toDegrees(shoulder);
        
        this.setElbowAngle(elbow);
        this.setShoulderAngle(shoulder);
    }

    public double getArmXPosition(){// 20.416                                         10.156
        double xPos = (Math.cos(Math.toRadians(getShoulderAngle()))*Constants.BICEP_LENGTH) + (Math.cos(Math.toRadians(getShoulderAngle()+getElbowAngle()))*Constants.FOREARM_LENGTH);
        
        return xPos;
    }

    public double getArmZPosition(){
        double zPos = (Math.sin(Math.toRadians(getShoulderAngle()))*Constants.BICEP_LENGTH) + (Math.sin(Math.toRadians(getShoulderAngle()+getElbowAngle()))*Constants.FOREARM_LENGTH);
        
        return zPos;
    }

}