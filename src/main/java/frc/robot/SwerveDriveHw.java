package frc.robot;


import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.interfaces.ISwerveDriveIo;

public class SwerveDriveHw implements ISwerveDriveIo {
    //measuring the robot, we got 11114 counts/90*, the theoretical amount is 10971.428/90* (150/7:1 gear ratio, 2048 counts/rev)
    private final double COUNTS_PER_DEGREE = 121.9; //using theoretical amount

    //measuring the robot, we got 13899 counts/rev, theoretical is 13824 counts/rev (L2 gearset at 6.75:1 ratio)
    //needs to be scaled * 39.37 (in/m) / (4"*Pi wheel diameter) / 10 (units per 100ms)
    private final double COUNTS_PER_METER = 4331.1;

    //motors and sensors
    private TalonFX turnMotor[];
    private TalonFX driveMotor[];
    private CANCoder absSensor[];
    private Pigeon2 pigeon;
    
    //sensor value buffers
    private double ypr_deg[];
    private double absSensorValue[];
    private double driveWheelVelocity[];
    private double driveWheelDistance[];
    private double turnMotorAngle[];
    private double absOffset[];

    private Translation2d[] swervePositions = {
        Constants.SWERVE_FRONT_LEFT_LOCATION,
        Constants.SWERVE_FRONT_RIGHT_LOCATION,
        Constants.SWERVE_BACK_LEFT_LOCATION,
        Constants.SWERVE_BACK_RIGHT_LOCATION
    };

    public SwerveDriveHw() {
        //initialize array sizes
        turnMotor = new TalonFX[Constants.NUM_WHEELS];
        driveMotor = new TalonFX[Constants.NUM_WHEELS];
        absSensor = new CANCoder[Constants.NUM_WHEELS];
        
        //initialize each motor/sensor
        turnMotor[ISwerveDrive.FL] = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR);
        turnMotor[ISwerveDrive.FR] = new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR);
        turnMotor[ISwerveDrive.RL] = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR);
        turnMotor[ISwerveDrive.RR] = new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR);

        driveMotor[ISwerveDrive.FL] = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR);
        driveMotor[ISwerveDrive.FR] = new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR);
        driveMotor[ISwerveDrive.RL] = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR);
        driveMotor[ISwerveDrive.RR] = new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR);

        absSensor[ISwerveDrive.FL] = new CANCoder(Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT);
        absSensor[ISwerveDrive.FR] = new CANCoder(Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT);
        absSensor[ISwerveDrive.RL] = new CANCoder(Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT);
        absSensor[ISwerveDrive.RR] = new CANCoder(Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT);

        absOffset = new double[Constants.NUM_WHEELS];
        absOffset[ISwerveDrive.FL] = Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET;
        absOffset[ISwerveDrive.FR] = Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET;
        absOffset[ISwerveDrive.RL] = Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET;
        absOffset[ISwerveDrive.RR] = Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET;

        pigeon = new Pigeon2(Constants.PIGEON_IMU_ID);

        TalonFXConfiguration allConfigs = new TalonFXConfiguration();

        for (CANCoder sensor: absSensor) {
            sensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 18);
            sensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 250);
        }

        for(TalonFX motor : driveMotor) {
            //motors MUST be reset every powerup!!!
            motor.configFactoryDefault();
            motor.getAllConfigs(allConfigs);
            allConfigs.slot0.kP = 0.02;
            allConfigs.slot0.kI = 0.0005;
            allConfigs.slot0.kD = 4;
            allConfigs.slot0.kF = 0.047;
            allConfigs.slot0.integralZone = 200;
            motor.configAllSettings(allConfigs);

            motor.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
        }

        for(TalonFX motor : turnMotor) {
            //motors MUST be reset every powerup!!!
            motor.configFactoryDefault();
            motor.getAllConfigs(allConfigs);
            allConfigs.slot1.kP = 0.2;
            allConfigs.slot1.kI = 0.0005;
            allConfigs.slot1.kD = 40;
            allConfigs.slot1.kF = 0;
            allConfigs.slot1.integralZone = 0;
            allConfigs.slot1.allowableClosedloopError = 300;
            allConfigs.motionCruiseVelocity = 20960;
            allConfigs.motionAcceleration = 40960;
            motor.configAllSettings(allConfigs);
            StatorCurrentLimitConfiguration cfg = new StatorCurrentLimitConfiguration();
            cfg.enable = false;
            cfg.currentLimit = 20;
            cfg.triggerThresholdCurrent = 40;
            motor.configStatorCurrentLimit(cfg);
            motor.selectProfileSlot(1, 0);

            motor.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
        }
        
        //initialize sensor buffers
        ypr_deg = new double[3];
        absSensorValue = new double[Constants.NUM_WHEELS];
        driveWheelVelocity = new double[Constants.NUM_WHEELS];
        driveWheelDistance = new double[Constants.NUM_WHEELS];
        turnMotorAngle = new double[Constants.NUM_WHEELS];

        Logger.RegisterTalon("FL Turn", turnMotor[ISwerveDrive.FL]);
        Logger.RegisterTalon("FR Turn", turnMotor[ISwerveDrive.FR]);
        Logger.RegisterTalon("RL Turn", turnMotor[ISwerveDrive.RL]);
        Logger.RegisterTalon("RR Turn", turnMotor[ISwerveDrive.RR]);

        Logger.RegisterTalon("FL Drive", driveMotor[ISwerveDrive.FL]);
        Logger.RegisterTalon("FR Drive", driveMotor[ISwerveDrive.FR]);
        Logger.RegisterTalon("RL Drive", driveMotor[ISwerveDrive.RL]);
        Logger.RegisterTalon("RR Drive", driveMotor[ISwerveDrive.RR]);

        Logger.RegisterCanCoder("FL Abs", absSensor[ISwerveDrive.FL]);
        Logger.RegisterCanCoder("FR Abs", absSensor[ISwerveDrive.FR]);
        Logger.RegisterCanCoder("RL Abs", absSensor[ISwerveDrive.RL]);
        Logger.RegisterCanCoder("RR Abs", absSensor[ISwerveDrive.RR]);

        Logger.RegisterSensor("FL Speed", () -> getCornerSpeed(ISwerveDrive.FL));
        Logger.RegisterSensor("FR Speed", () -> getCornerSpeed(ISwerveDrive.FR));
        Logger.RegisterSensor("RL Speed", () -> getCornerSpeed(ISwerveDrive.RL));
        Logger.RegisterSensor("RR Speed", () -> getCornerSpeed(ISwerveDrive.RR));

        Logger.RegisterSensor("FL Turn Pos", () -> getCornerAngle(ISwerveDrive.FL));
        Logger.RegisterSensor("FR Turn Pos", () -> getCornerAngle(ISwerveDrive.FR));
        Logger.RegisterSensor("RL Turn Pos", () -> getCornerAngle(ISwerveDrive.RL));
        Logger.RegisterSensor("RR Turn Pos", () -> getCornerAngle(ISwerveDrive.RR));

        Logger.RegisterPigeon(pigeon);
    }

    @Override
    public void updateInputs() {
        pigeon.getYawPitchRoll(ypr_deg);

        for(int i=0; i<Constants.NUM_WHEELS; i++) {
            absSensorValue[i] = absSensor[i].getAbsolutePosition();
            driveWheelVelocity[i] = driveMotor[i].getSelectedSensorVelocity() / COUNTS_PER_METER;
            driveWheelDistance[i] = driveMotor[i].getSelectedSensorPosition() / COUNTS_PER_METER;
            turnMotorAngle[i] = -turnMotor[i].getSelectedSensorPosition() / COUNTS_PER_DEGREE;
        }
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(ypr_deg[0]);
    }

    @Override
    public double getPitch() {
        return ypr_deg[1];
    }

    @Override
    public double getRoll() {
        return ypr_deg[2];
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        NeutralMode mode;

        if(brakeOn) {
            mode = NeutralMode.Brake;
        } else {
            mode = NeutralMode.Coast;
        }

        for (TalonFX motor : turnMotor) {
            motor.setNeutralMode(mode);
        }
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        NeutralMode mode;

        if(brakeOn) {
            mode = NeutralMode.Brake;
        } else {
            mode = NeutralMode.Coast;
        }
        
        for (TalonFX motor : driveMotor) {
            motor.setNeutralMode(mode);
        }
    }

    @Override
    public double getCornerAbsAngle(int wheel) {
        return absSensorValue[wheel];
    }

    @Override
    public double getCornerAngle(int wheel) {
        return turnMotorAngle[wheel];
    }

    @Override
    public double getCornerSpeed(int wheel) {
        return driveWheelVelocity[wheel];
    }

    @Override
    public void setDriveCommand(int wheel, ControlMode mode, double output) {
        driveMotor[wheel].set(mode, output);
    }

    @Override
    public void setTurnCommand(int wheel, ControlMode mode, double output) {
        turnMotor[wheel].set(mode, output);
    }

    @Override
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        driveMotor[wheel].set(ControlMode.Velocity, swerveModuleState.speedMetersPerSecond * COUNTS_PER_METER);
        turnMotor[wheel].set(ControlMode.MotionMagic, swerveModuleState.angle.getDegrees() * COUNTS_PER_DEGREE);
    }

    @Override
    public void setKinematics(SwerveDriveKinematics kinematics) {
        //not needed
    }

    @Override
    public double getCornerDistance(int wheel) {
        return driveWheelDistance[wheel];
    }

    @Override
    public Translation2d[] getCornerLocations() {
        return swervePositions;
    }

    @Override
    public double getWheelOffset(int wheel) {
        return absOffset[wheel];
    }
    
}
