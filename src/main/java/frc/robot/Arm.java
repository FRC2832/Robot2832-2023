package frc.robot;

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
        SmartDashboard.putNumber("Arm X", getArmXPosition());
        SmartDashboard.putNumber("Arm Z", getArmZPosition());
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
        boolean side = (x > -46); //arm cannot reach outside of 48 inches, we have a grabber on the arm so it checks for <46
        boolean armLength = Math.sqrt((x*x) + (z*z)) < 60; //total armLength for 34in plus 26in00934
        if(side && armLength){
            double l = Math.abs(x);
            double h = Math.sqrt(l * l + z * z);
            double phi = Math.toDegrees(Math.atan(z/l));
            double theta = Math.toDegrees(Math.acos((h/2)/30));
            if(x >= 0) {
                shoulder = phi + theta;
                elbow = (phi - theta);
            }
            else{
                shoulder = 180 - (phi + theta);
                elbow = 180 - (phi - theta);
            }
        }
        else { //if requested point is outside limited range then set arm angles to what they were before
            elbow = getElbowAngle();
            shoulder = getShoulderAngle();
        }
        this.setElbowAngle(elbow);
        this.setShoulderAngle(shoulder);
    }

    public double getArmXPosition(){
        double xPos = (Math.cos(getShoulderAngle())*Constants.BICEP_LENGTH) + (Math.cos(getShoulderAngle()+getElbowAngle())*Constants.FOREARM_LENGTH);
        return xPos;
    }

    public double getArmZPosition(){
        double zPos = (Math.sin(getShoulderAngle())*Constants.BICEP_LENGTH) + (Math.sin(getShoulderAngle()+getElbowAngle())*Constants.FOREARM_LENGTH);
        return zPos;
    }

}