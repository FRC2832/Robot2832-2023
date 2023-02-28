package frc.robot;

import org.livoniawarriors.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.interfaces.IArmControl;

public class Arm implements Subsystem{
    private IArmControl hardware;
    private double shoulderAng;
    private double elbowAng;
    
    PIDController shoulderPid;
    ProfiledPIDController elbowPid;
    boolean shoulderPIDRan;
    boolean elbowPIDRan;

    public Arm(IArmControl hardware) {
        this.hardware = hardware;
        hardware.updateInputs();

        Logger.RegisterSensor("Shoulder Angle", () -> getShoulderAngle());
        Logger.RegisterSensor("Elbow Angle", () -> getElbowAngle());

        shoulderPid = new PIDController(.1, 0.002, 0);
        elbowPid = new ProfiledPIDController(.2, 0.002, 0,
            new Constraints(120, 120));  //units are in deg/s and deg/s^2
        elbowPid.setTolerance(2);
        elbowPid.reset(getElbowAngle());
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
        SmartDashboard.putNumber("Arm X", getArmXPosition());
        SmartDashboard.putNumber("Arm Z", getArmZPosition());
        hardware.checkBrake();
    }

    public void setShoulderAngle(double angleDeg) {
        double shoulderAngle = getShoulderAngle();
        double volts = shoulderPid.calculate(shoulderAngle, angleDeg);
        double delta = Math.abs(shoulderAngle - angleDeg);
        if(delta>4){
            shoulderPIDRan = true;
        }
        if(delta < 2 && shoulderPIDRan){
            shoulderPIDRan = false;
        }
        if(Math.abs(volts) > 4){
            volts = Math.signum(volts) * 4;
        }
        if(!shoulderPIDRan){
            volts = 0;
        }
        setShoulderMotorVolts(volts);
        SmartDashboard.putNumber("Shoulder Angle Command", angleDeg);
        SmartDashboard.putNumber("Shoulder Volts Command", volts);
    }

    public void setElbowAngle(double angleDeg) {
        elbowPid.setGoal(angleDeg);
        double ff = hardware.getFeedForward(getShoulderAngle());
        double volts = -elbowPid.calculate(getElbowAngle()) + ff;

        //if(elbowPid.atSetpoint()){
        //    volts = 0;
        //}
        setElbowMotorVolts(volts);
        SmartDashboard.putNumber("Elbow Angle Command", angleDeg);
        SmartDashboard.putNumber("Elbow Volts Command", volts);
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

    public void resetPids() {
        elbowPid.reset(getElbowAngle());
        shoulderPid.reset();
    }

    //this one will become not good once we figure out the equation for an arm with two segments of different lengths
    public void calcAngles(double x, double z) { //calculate the angles for each part of the arm to get to the point (x, z)
        // from https://www.youtube.com/watch?v=Q-UeYEpwXXU
        SmartDashboard.putNumber("Commanded Arm X", x);
        SmartDashboard.putNumber("Commanded Arm Z", z);
        
        double shoulder = 0;
        double elbow = 0;
        double forearmLen = Constants.FOREARM_LENGTH;
        double bicepLen = Constants.BICEP_LENGTH;

        boolean sideLimit = x < 77.5 && x > -48;
        boolean heightLimit = z < 59 && z > -14;
        boolean robotHeightLimit = (-5 < x && x < 30) && (z < 0);
        boolean armLengthLimit = Math.sqrt(x*x + z*z) < forearmLen + bicepLen;

        boolean limits = sideLimit && heightLimit && !robotHeightLimit && armLengthLimit;

        //40-45 finds elbow angle in radians, but havent tested to see which angle is for the up reaching arm and down reaching arm
        if(x > 0 && limits) {
            elbow =  Math.acos((bicepLen*bicepLen + forearmLen*forearmLen - x*x - z*z)/(2*bicepLen*forearmLen)) - 3.14159;
            shoulder = Math.atan(z/x) - Math.atan((forearmLen*Math.sin(elbow))/(bicepLen + forearmLen*Math.cos(elbow)));
        }
        else if(x < 0 && limits){
            elbow = Math.acos((x*x + z*z - bicepLen*bicepLen - forearmLen*forearmLen)/(2*bicepLen*forearmLen));
            shoulder = 3.14159 - Math.atan(z/Math.abs(x)) - Math.atan((forearmLen*Math.sin(elbow))/(bicepLen + forearmLen*Math.cos(elbow)));
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
