package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
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
        //shoulderMotor.set(ControlMode.Position, angleDeg * COUNTS_PER_DEGREE_SHOULDER);
        SmartDashboard.putNumber("Shoulder Angle Command", angleDeg);
    }

    @Override
    public void setElbowAngle(double angleDeg) {
        // TODO Implement ARM PID
        //elbowMotor.set(ControlMode.Position, angleDeg * COUNTS_PER_DEGREE_ELBOW);
        SmartDashboard.putNumber("Elbow Angle Command", angleDeg);
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
            brakes.Brake(false, 1);
        }
        else {
            brakes.Brake(true, 1);
        }
        if(shoulder < 0.03){
            brakes.Brake(false, 0);
        }
        else {
            brakes.Brake(true, 0);
        }
    }

    @Override
    public void updateInputs() {
        
        //Frac is 0 at lowest point, 1 at max extension
        var rawDC = shoulderEncoder.getOutput();
        SmartDashboard.putNumber("Shoulder Raw", rawDC);
        //make shoulder between -90 to 270 to "ignore" the rollover point at vertical on the arm
        shoulderAngle = MathUtil.inputModulus((rawDC * 360) - 287,-90,270);

        shoulderMotor.setSelectedSensorPosition(shoulderAngle * COUNTS_PER_DEGREE_SHOULDER);

        rawDC = elbowEncoder.getOutput();
        SmartDashboard.putNumber("Elbow Raw", rawDC);
        elbowAngle =  MathUtil.inputModulus((rawDC * 360)-135,-180,180);

        elbowMotor.setSelectedSensorPosition(elbowAngle * COUNTS_PER_DEGREE_ELBOW);

    }
    
}
