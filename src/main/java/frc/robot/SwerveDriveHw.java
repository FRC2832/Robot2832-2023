package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private PigeonIMU pigeon;
    private Orchestra siren;
    
    //sensor value buffers
    private double ypr_deg[];
    private double absSensorValue[];
    private double driveWheelVelocity[];
    private double turnMotorAngle[];

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

        pigeon = new PigeonIMU(Constants.PIGEON_IMU_ID);

        TalonFXConfiguration allConfigs = new TalonFXConfiguration();

        for(TalonFX motor : driveMotor) {
            motor.getAllConfigs(allConfigs);
            allConfigs.slot0.kP = 0.02;
            allConfigs.slot0.kI = 0.0005;
            allConfigs.slot0.kD = 4;
            allConfigs.slot0.kF = 0.047;
            allConfigs.slot0.integralZone = 200;
            motor.configAllSettings(allConfigs);
        }

        for(TalonFX motor : turnMotor) {
            motor.getAllConfigs(allConfigs);
            allConfigs.slot0.kP = 0.1;
            allConfigs.slot0.kI = 0.0015;
            allConfigs.slot0.kD = 10;
            allConfigs.slot0.kF = 0;
            allConfigs.slot0.integralZone = 900;
            allConfigs.slot0.allowableClosedloopError = 90;
            motor.configAllSettings(allConfigs);
        }

        siren = new Orchestra();
        siren.loadMusic("Warning_Siren.chrp");
        for(TalonFX motor : turnMotor) {
            siren.addInstrument(motor);
        }
        
        //initialize sensor buffers
        ypr_deg = new double[3];
        absSensorValue = new double[Constants.NUM_WHEELS];
        driveWheelVelocity = new double[Constants.NUM_WHEELS];
        turnMotorAngle = new double[Constants.NUM_WHEELS];

        SmartDashboard.putNumber("Angle", 0);
    }

    @Override
    public void updateInputs() {
        pigeon.getYawPitchRoll(ypr_deg);

        for(int i=0; i<Constants.NUM_WHEELS; i++) {
            absSensorValue[i] = absSensor[i].getAbsolutePosition();
            driveWheelVelocity[i] = driveMotor[i].getSelectedSensorVelocity() / COUNTS_PER_METER;
            turnMotorAngle[i] = -turnMotor[i].getSelectedSensorPosition() / COUNTS_PER_DEGREE;
        }
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(ypr_deg[0]);
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
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        driveMotor[wheel].set(ControlMode.Velocity, swerveModuleState.speedMetersPerSecond * COUNTS_PER_METER);
        turnMotor[wheel].set(ControlMode.Position, swerveModuleState.angle.getDegrees() * COUNTS_PER_DEGREE);

        //var counts = SmartDashboard.getNumber("Angle", 0);
        //turnMotor[3].set(ControlMode.Position, counts * COUNTS_PER_DEGREE);
        /*double time = Timer.getFPGATimestamp();
        int active = ((int)time) % 4;
        if (active == 1) {
            turnMotor[wheel].set(ControlMode.MusicTone, 440);
        } else if (active == 2) {
            turnMotor[wheel].set(ControlMode.MusicTone, 220);
        } else  {
            turnMotor[wheel].set(ControlMode.MusicTone, 0);
        }*/
    }

    @Override
    public void setKinematics(SwerveDriveKinematics kinematics) {
        //not needed
    }
    
}
