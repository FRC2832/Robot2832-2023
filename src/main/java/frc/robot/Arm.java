package frc.robot;

import org.livoniawarriors.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.IArmControl;

public class Arm extends SubsystemBase{
    private IArmControl hardware;
    private double shoulderAng;
    private double elbowAng;
    
    ProfiledPIDController shoulderPid;
    ProfiledPIDController elbowPid;
    boolean shoulderPIDRan;
    boolean elbowPIDRan;

    private double shoulderDelta;
    private double elbowDelta;

    public Arm(IArmControl hardware) {
        super();
        this.hardware = hardware;
        hardware.updateInputs();

        Logger.RegisterSensor("Shoulder Angle", () -> getShoulderAngle());
        Logger.RegisterSensor("Elbow Angle", () -> getElbowAngle());

        shoulderPid = new ProfiledPIDController(.2, 0.002, 0, 
            new Constraints(90, 90));
        shoulderPid.setTolerance(0.5);
        elbowPid = new ProfiledPIDController(.2, 0.002, 0,
            new Constraints(150, 100));  //units are in deg/s and deg/s^2
        elbowPid.setTolerance(0.5);
        resetPids();
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
        SmartDashboard.putNumber("Abs Arm X", getArmXPosition());
        SmartDashboard.putNumber("Abs Arm Z", getArmZPosition());

        SmartDashboard.putNumber("Rel Arm X", getArmXPosition(hardware.getShoulderAngle(), hardware.getElbowAngle()));
        SmartDashboard.putNumber("Rel Arm Z", getArmZPosition(hardware.getShoulderAngle(), hardware.getElbowAngle()));

        shoulderDelta = hardware.getShoulderAbsAngle() - hardware.getShoulderAngle();
        elbowDelta = hardware.getElbowAbsAngle() - hardware.getElbowAngle();

        SmartDashboard.putNumber("Shoulder Delta", shoulderDelta);
        SmartDashboard.putNumber("Elbow Delta", elbowDelta);

        hardware.checkBrake();
    }

    public void setShoulderAngle(double angleDeg) {
        //shoulderPid.setGoal(angleDeg);
        //double volts = shoulderPid.calculate(getShoulderAngle());

        //setShoulderMotorVolts(volts);
        var newAngle = angleDeg - shoulderDelta;
        hardware.setShoulderAngle(newAngle);
        SmartDashboard.putNumber("Shoulder Angle Command", newAngle);
        //SmartDashboard.putNumber("Shoulder Volts Command", volts);
    }

    public void setElbowAngle(double angleDeg) {
        var newAngle = angleDeg - elbowDelta;
        hardware.setElbowAngle(newAngle);
        SmartDashboard.putNumber("Elbow Angle Command", newAngle);
        //SmartDashboard.putNumber("Elbow Volts Command", volts);
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
        shoulderPid.reset(getShoulderAngle());
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

        boolean sideLimit = x < 77.5 && x > -41.1;
        boolean heightLimit = z < 59 && z > -14;
        boolean robotHeightLimit = (-5 < x && x < 30) && (z < 0);
        boolean armLengthLimit = Math.sqrt(x*x + z*z) < forearmLen + bicepLen;

        //boolean limits = sideLimit && heightLimit && !robotHeightLimit && armLengthLimit;       
        var limits = true;
        SmartDashboard.putBoolean("Arm Limit Hit", limits);

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

        SmartDashboard.putNumber("Commanded Shoulder", shoulder);
        SmartDashboard.putNumber("Commanded Elbow", elbow);
    }

    public double getArmXPosition() {
        return getArmXPosition(hardware.getShoulderAbsAngle(), hardware.getElbowAbsAngle());
    }

    public double getArmZPosition() {
        return getArmZPosition(hardware.getShoulderAbsAngle(), hardware.getElbowAbsAngle());
    }

    public double getArmXPosition(double shoulderAngle, double elbowAngle) {
        return (Math.cos(Math.toRadians(shoulderAngle))*Constants.BICEP_LENGTH) + (Math.cos(Math.toRadians(shoulderAngle + elbowAngle))*Constants.FOREARM_LENGTH);
    }

    public double getArmZPosition(double shoulderAngle, double elbowAngle) {
        return (Math.sin(Math.toRadians(shoulderAngle))*Constants.BICEP_LENGTH) + (Math.sin(Math.toRadians(shoulderAngle + elbowAngle))*Constants.FOREARM_LENGTH);
    }
}
