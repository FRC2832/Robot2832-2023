package frc.robot;

import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IArmControl;

public class ArmHw implements IArmControl {
    TalonFX shoulderMotor;
    TalonFX elbowMotor;
    DutyCycle shoulderEncoder;
    double shoulderAngle;
    DutyCycle elbowEncoder;
    double elbowAngle;
    ArmBrakes brakes;
    PIDController shoulderPid;
    PIDController elbowPid;
    boolean shoulderPIDRan;
    boolean elbowPIDRan;

    final double COUNTS_PER_DEGREE_SHOULDER = 152.492;
    final double COUNTS_PER_DEGREE_ELBOW =  426.667;

    public ArmHw() {
        shoulderMotor = new TalonFX(46);
        shoulderMotor.setNeutralMode(NeutralMode.Brake);
        elbowMotor = new TalonFX(45);
        elbowMotor.setNeutralMode(NeutralMode.Brake);
        shoulderEncoder = new DutyCycle(new DigitalInput(0));
        elbowEncoder = new DutyCycle(new DigitalInput(1));
        brakes = new ArmBrakes();

        shoulderPid = new PIDController(.6, 0.002, 0); // originally kp = .3; ki = 0
        elbowPid = new PIDController(.6, 0.002, 0);

        Logger.RegisterTalon("Shoulder", shoulderMotor);
        Logger.RegisterTalon("Elbow", elbowMotor);
        Logger.RegisterSensor("Shoulder Angle", () -> getShoulderAngle());
        Logger.RegisterSensor("Elbow Angle", () -> getElbowAngle());
    }

    @Override
    public void setShoulderAngle(double angleDeg) {
        double volts = shoulderPid.calculate(shoulderAngle, angleDeg);
        double delta = Math.abs(shoulderAngle - angleDeg);
        if(delta>4){
            shoulderPIDRan = true;
        }
        if(delta < 2 && shoulderPIDRan){
            shoulderPIDRan = false;
        }
        if(Math.abs(volts) > 3){
            volts = Math.signum(volts) * 3;
        }
        if(!shoulderPIDRan){
            volts = 0;
        }
        setShoulderMotorVolts(volts);
        SmartDashboard.putNumber("Shoulder Angle Command", angleDeg);
        SmartDashboard.putNumber("Shoulder Volts Command", volts);
    }

    @Override
    public void setElbowAngle(double angleDeg) {
        double volts = -elbowPid.calculate(elbowAngle, angleDeg);
        double delta = Math.abs(elbowAngle - angleDeg);
        if(delta>4){
            elbowPIDRan = true;
        }
        if(delta < 2 && elbowPIDRan){
            elbowPIDRan = false;
        }
        if(Math.abs(volts) > 3){
            volts = Math.signum(volts) * 3;
        }
        if(!elbowPIDRan){
            volts = 0;
        }
        setElbowMotorVolts(volts);
        SmartDashboard.putNumber("Elbow Angle Command", angleDeg);
        SmartDashboard.putNumber("Elbow Volts Command", volts);
    }

    @Override
    public void setShoulderMotorVolts(double volts) {
        shoulderMotor.set(ControlMode.PercentOutput, volts/Constants.NOM_BATTERY_VOLTAGE);
    }

    @Override
    public void setElbowMotorVolts(double volts) {
        elbowMotor.set(ControlMode.PercentOutput, volts/Constants.NOM_BATTERY_VOLTAGE);
    }

    @Override
    public double getElbowAngle() {
        return elbowAngle;
    }

    @Override
    public double getShoulderAngle() {
        return shoulderAngle;
    }

    @Override
    public void checkBrake(){
        double elbow = Math.abs(elbowMotor.getMotorOutputPercent());
        double shoulder = Math.abs(shoulderMotor.getMotorOutputPercent());
        if(elbow < 0.03){
            brakes.Brake(false, 2);
        }
        else {
            brakes.Brake(true, 2);
        }
        if(shoulder < 0.03){
            brakes.Brake(false, 3);
        }
        else {
            brakes.Brake(true, 3);
        }
    }

    @Override
    public void updateInputs() {
        
        //Frac is 0 at lowest point, 1 at max extension
        var rawDC = shoulderEncoder.getOutput();
        //make shoulder between -90 to 270 to "ignore" the rollover point at vertical on the arm
        shoulderAngle = MathUtil.inputModulus((rawDC * 360) - Constants.SHOULDER_OFFSET,-90,270);

        shoulderMotor.setSelectedSensorPosition(shoulderAngle * COUNTS_PER_DEGREE_SHOULDER);

        rawDC = elbowEncoder.getOutput();
        elbowAngle =  MathUtil.inputModulus((rawDC * 360) - Constants.ELBOW_OFFSET,-180,180);

        elbowMotor.setSelectedSensorPosition(elbowAngle * COUNTS_PER_DEGREE_ELBOW);

    }
    
}
