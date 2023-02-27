package frc.robot;

import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.interfaces.IArmControl;

public class ArmHw implements IArmControl {
    TalonFX shoulderMotor;
    TalonFX elbowMotor;
    DutyCycle shoulderEncoder;
    double shoulderAngle;
    DutyCycle elbowEncoder;
    double elbowAngle;
    double elbowAngleParallel;
    ArmBrakes brakes;

    final double COUNTS_PER_DEGREE_SHOULDER = 152.492;
    final double COUNTS_PER_DEGREE_ELBOW =  426.667;

    private final double kF_ELBOW = 0.816; //12.55 bat * 6.5% power to keep flat

    public ArmHw() {
        shoulderMotor = new TalonFX(46);
        shoulderMotor.setNeutralMode(NeutralMode.Brake);
        elbowMotor = new TalonFX(45);
        elbowMotor.setNeutralMode(NeutralMode.Brake);
        shoulderEncoder = new DutyCycle(new DigitalInput(0));
        elbowEncoder = new DutyCycle(new DigitalInput(1));
        brakes = new ArmBrakes();

        Logger.RegisterTalon("Shoulder", shoulderMotor);
        Logger.RegisterTalon("Elbow", elbowMotor);
    }

    @Override
    public void setShoulderMotorVolts(double volts) {
        shoulderMotor.set(ControlMode.PercentOutput, volts/RobotController.getBatteryVoltage());
    }

    @Override
    public void setElbowMotorVolts(double volts) {
        elbowMotor.set(ControlMode.PercentOutput, volts/RobotController.getBatteryVoltage());
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
    

    @Override
    public double getFeedForward(double offset) {
        return Math.cos(Math.toRadians(offset + elbowAngle)) * kF_ELBOW;
    }
}
