package frc.robot;

import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    //75:1 gearbox, 72/30 shoulder gear, 2048 pulses per rev
    final double COUNTS_PER_DEGREE_SHOULDER = 64/1 * 76/30 * 2048 / 360;
    final double COUNTS_PER_DEGREE_SHOULDER_V = COUNTS_PER_DEGREE_SHOULDER / 10;

    //75:1 gearbox, 60:30 pulley, 2048 pulses per rev
    final double COUNTS_PER_DEGREE_ELBOW = 60/1 * 60/30 * 2048 / 360;
    final double COUNTS_PER_DEGREE_ELBOW_V = COUNTS_PER_DEGREE_ELBOW / 10;

    private final double kF_ELBOW = 0.716; //12.55 bat * 6.5% power to keep flat
    private final double kF_SHOULDER = 0.952; //12.8 * 9% to keep arm up

    public ArmHw() {
        TalonFXConfiguration allConfigs = new TalonFXConfiguration();

        shoulderMotor = new TalonFX(46);
        shoulderMotor.setNeutralMode(NeutralMode.Brake);
        shoulderMotor.setInverted(true);

        //motors MUST be reset every powerup!!!
        shoulderMotor.configFactoryDefault();
        shoulderMotor.setNeutralMode(NeutralMode.Brake);
        shoulderMotor.setInverted(true);

        shoulderMotor.getAllConfigs(allConfigs);
        allConfigs.slot0.kP = 0.12 / COUNTS_PER_DEGREE_SHOULDER * Constants.CTRE_P_RES;
        allConfigs.slot0.kI = 1.603e-5;    //TODO: Try later... 0.08 / COUNTS_PER_DEGREE_SHOULDER;
        allConfigs.slot0.kD = 0;
        allConfigs.slot0.kF = 0;
        allConfigs.slot0.integralZone = 10 * COUNTS_PER_DEGREE_SHOULDER;
        allConfigs.slot0.allowableClosedloopError = 0;
        allConfigs.motionCruiseVelocity = 80 * COUNTS_PER_DEGREE_SHOULDER_V;
        allConfigs.motionAcceleration = 70 * COUNTS_PER_DEGREE_SHOULDER_V;
        shoulderMotor.configAllSettings(allConfigs);
        shoulderMotor.getIntegralAccumulator();
        shoulderMotor.getErrorDerivative();
        shoulderMotor.getClosedLoopError();

        elbowMotor = new TalonFX(45);
        //motors MUST be reset every powerup!!!
        elbowMotor.configFactoryDefault();
        elbowMotor.setNeutralMode(NeutralMode.Brake);
        elbowMotor.setInverted(true);

        elbowMotor.getAllConfigs(allConfigs);
        allConfigs.slot0.kP = 0.2 / COUNTS_PER_DEGREE_ELBOW * Constants.CTRE_P_RES;
        allConfigs.slot0.kI = 1.603e-5;
        allConfigs.slot0.kD = 0;
        allConfigs.slot0.kF = 0;
        allConfigs.slot0.integralZone = 10 * COUNTS_PER_DEGREE_ELBOW;
        allConfigs.slot0.allowableClosedloopError = 0;
        allConfigs.motionCruiseVelocity = 100 * COUNTS_PER_DEGREE_ELBOW_V;
        allConfigs.motionAcceleration = 200 * COUNTS_PER_DEGREE_ELBOW_V;
        elbowMotor.configAllSettings(allConfigs);

        shoulderEncoder = new DutyCycle(new DigitalInput(0));
        elbowEncoder = new DutyCycle(new DigitalInput(1));
        brakes = new ArmBrakes();

        Logger.RegisterTalon("Shoulder", shoulderMotor);
        Logger.RegisterTalon("Elbow", elbowMotor);

        Logger.RegisterSensor("Shoulder Motor Degrees", () -> shoulderMotor.getSelectedSensorPosition() / COUNTS_PER_DEGREE_SHOULDER);
        Logger.RegisterSensor("Elbow Motor Degrees", () -> elbowMotor.getSelectedSensorPosition() / COUNTS_PER_DEGREE_ELBOW);

        Logger.RegisterSensor("Shoulder Sensor Degrees", () -> shoulderAngle);
        Logger.RegisterSensor("Elbow Sensor Degrees", () -> elbowAngle);

        Logger.RegisterSensor("Shoulder Motor Velocity", () -> shoulderMotor.getSelectedSensorVelocity() / COUNTS_PER_DEGREE_SHOULDER_V);
        Logger.RegisterSensor("Elbow Motor Velocity", () -> elbowMotor.getSelectedSensorVelocity() / COUNTS_PER_DEGREE_ELBOW_V);
    }

    @Override
    public void setShoulderMotorVolts(double volts) {
        shoulderMotor.set(ControlMode.PercentOutput, volts/RobotController.getBatteryVoltage());
    }

    @Override
    public void setElbowMotorVolts(double volts) {
        elbowMotor.set(ControlMode.PercentOutput, -volts/RobotController.getBatteryVoltage());
    }

    @Override
    public double getElbowAngle() {
        //return elbowAngle;
        return elbowMotor.getSelectedSensorPosition() / COUNTS_PER_DEGREE_ELBOW;
    }

    @Override
    public double getShoulderAngle() {
        //return shoulderAngle;
        return shoulderMotor.getSelectedSensorPosition() / COUNTS_PER_DEGREE_SHOULDER;
    }

    @Override
    public void checkBrake(){
        double elbow = Math.abs(elbowMotor.getMotorOutputPercent());
        double shoulder = Math.abs(shoulderMotor.getMotorOutputPercent());
        if(elbow < 0.045){
            brakes.Brake(false, 2);
        }
        else {
            brakes.Brake(true, 2);
        }
        if(shoulder < 0.045
        ){
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
        
        rawDC = elbowEncoder.getOutput();
        elbowAngle = MathUtil.inputModulus((rawDC * 360) - Constants.ELBOW_OFFSET,-180,180);

        if(DriverStation.isDisabled()) {
            shoulderMotor.setSelectedSensorPosition(shoulderAngle * COUNTS_PER_DEGREE_SHOULDER);
            elbowMotor.setSelectedSensorPosition(elbowAngle * COUNTS_PER_DEGREE_ELBOW);
        }

        SmartDashboard.putNumber("Shoulder Error I",shoulderMotor.getIntegralAccumulator());
        SmartDashboard.putNumber("Shoulder Error D",shoulderMotor.getErrorDerivative());
        SmartDashboard.putNumber("Shoulder Error P",shoulderMotor.getClosedLoopError());
        SmartDashboard.putNumber("Elbow Error I",elbowMotor.getIntegralAccumulator());
        SmartDashboard.putNumber("Elbow Error D",elbowMotor.getErrorDerivative());
        SmartDashboard.putNumber("Elbow Error P",elbowMotor.getClosedLoopError());
    }
    
    @Override
    public double getFeedForward(double offset) {
        return Math.cos(Math.toRadians(offset + elbowAngle)) * kF_ELBOW;
    }

    @Override
    public void setElbowAngle(double angleDeg) {
        elbowMotor.set(ControlMode.MotionMagic, angleDeg * COUNTS_PER_DEGREE_ELBOW, 
            DemandType.ArbitraryFeedForward, getFeedForward(shoulderAngle)/RobotController.getBatteryVoltage());
    }

    @Override
    public void setShoulderAngle(double angleDeg) {
        var ff = Math.cos(Math.toRadians(shoulderAngle)) * kF_SHOULDER;
        shoulderMotor.set(ControlMode.MotionMagic, angleDeg * COUNTS_PER_DEGREE_SHOULDER,
            DemandType.ArbitraryFeedForward, ff/RobotController.getBatteryVoltage());
    }
}
