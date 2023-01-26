package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

    final double MIN_DUTY_CYCLE = 0.998;
    final double MAX_DUTY_CYCLE = 0.167;
    final double SHOULDER_MIN_ANGLE = -55.7;
    final double SHOULDER_MAX_ANGLE = 180+65.3;
    final double ELBOW_MIN_ANGLE = -55.7;
    final double ELBOW_MAX_ANGLE = 180+65.3;

    public ArmHw() {
        shoulderMotor = new TalonFX(60);
        elbowMotor = new TalonFX(61);
        shoulderEncoder = new DutyCycle(new DigitalInput(2));
    }

    @Override
    public void setShoulderAngle(double angleDeg) {
        // TODO Implement ARM PID
        
    }

    @Override
    public void setElbowAngle(double angleDeg) {
        // TODO Implement ARM PID
        
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
    public void updateInputs() {
        
        //Frac is 0 at lowest point, 1 at max extension
        var rawDC = shoulderEncoder.getOutput();
        var dcFrac = (rawDC - MIN_DUTY_CYCLE) / (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);
        shoulderAngle = SHOULDER_MIN_ANGLE + dcFrac * (SHOULDER_MAX_ANGLE - SHOULDER_MIN_ANGLE);

        rawDC = elbowEncoder.getOutput();
        dcFrac = (rawDC - MIN_DUTY_CYCLE) / (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);
        elbowAngle = ELBOW_MIN_ANGLE + dcFrac * (ELBOW_MAX_ANGLE - ELBOW_MIN_ANGLE);
    }
    
}
