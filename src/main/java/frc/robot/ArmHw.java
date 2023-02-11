package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.interfaces.IArmControl;

public class ArmHw implements IArmControl {
    TalonFX shoulderMotor;
    TalonFX elbowMotor;
    DutyCycle shoulderEncoder;
    double shoulderAngle;
    DutyCycle elbowEncoder;
    double elbowAngle;
    ArmBrakes brakes;

    final double MIN_DUTY_CYCLE = 0.998;
    final double MAX_DUTY_CYCLE = 0.167;
    final double SHOULDER_MIN_ANGLE = -55.7;
    final double SHOULDER_MAX_ANGLE = 180+65.3;
    final double ELBOW_MIN_ANGLE = -55.7;
    final double ELBOW_MAX_ANGLE = 180+65.3;
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
    }

    @Override
    public void setShoulderAngle(double angleDeg) {
        // TODO Implement ARM PID
        shoulderMotor.set(ControlMode.Position, angleDeg * COUNTS_PER_DEGREE_SHOULDER);
    }

    @Override
    public void setElbowAngle(double angleDeg) {
        // TODO Implement ARM PID
        elbowMotor.set(ControlMode.Position, angleDeg * COUNTS_PER_DEGREE_ELBOW);
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
        double elbow = elbowMotor.getMotorOutputPercent();
        double shoulder = shoulderMotor.getMotorOutputPercent();
        if(elbow < 3){
            brakes.Brake(true, 1);
        }
        else {
            brakes.Brake(false, 1);
        }
        if(shoulder < 3){
            brakes.Brake(true, 0);
        }
        else {
            brakes.Brake(false, 0);
        }
    }

    @Override
    public void updateInputs() {
        
        //Frac is 0 at lowest point, 1 at max extension
        var rawDC = shoulderEncoder.getOutput();
        var dcFrac = (rawDC - MIN_DUTY_CYCLE) / (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);
        shoulderAngle = SHOULDER_MIN_ANGLE + dcFrac * (SHOULDER_MAX_ANGLE - SHOULDER_MIN_ANGLE);

        shoulderMotor.setSelectedSensorPosition(shoulderAngle * COUNTS_PER_DEGREE_SHOULDER);

        rawDC = elbowEncoder.getOutput();
        dcFrac = (rawDC - MIN_DUTY_CYCLE) / (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);
        elbowAngle = ELBOW_MIN_ANGLE + dcFrac * (ELBOW_MAX_ANGLE - ELBOW_MIN_ANGLE);

        elbowMotor.setSelectedSensorPosition(elbowAngle * COUNTS_PER_DEGREE_ELBOW);

    }
    
}
