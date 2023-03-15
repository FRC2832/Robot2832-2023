package frc.robot.simulation;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.interfaces.ISwerveDriveIo;

public class SwerveDriveSim implements ISwerveDriveIo {
    private SwerveDriveKinematics kinematics;
    private double chassisAngle;
    private double pitchAngle;

    private double absAngle[];
    private double absOffset[];
    private double turnAngle[];
    private double driveSpeed[];
    private double driveDist[];
    private ControlMode driveCommand[];
    private ControlMode turnCommand[];
    private double drivePower[];
    private double turnPower[];

    private FlywheelSim turnMotorSim[];
    private PIDController turningPIDController[];

    private final double Kv_Turn = 0.006531;

    private Translation2d[] swervePositions = {
        new Translation2d(0.291, 0.291),
        new Translation2d(0.291, -0.291),
        new Translation2d(-0.291, 0.291),
        new Translation2d(-0.291, -0.291),
    };

    public SwerveDriveSim() {
        absAngle = new double[Constants.NUM_WHEELS];
        absOffset = new double[Constants.NUM_WHEELS];
        turnAngle = new double[Constants.NUM_WHEELS];
        driveSpeed = new double[Constants.NUM_WHEELS];
        driveDist = new double[Constants.NUM_WHEELS];

        driveCommand = new ControlMode[Constants.NUM_WHEELS];
        turnCommand = new ControlMode[Constants.NUM_WHEELS];
        drivePower = new double[Constants.NUM_WHEELS];
        turnPower = new double[Constants.NUM_WHEELS];

        chassisAngle = 0;

        absOffset[ISwerveDrive.FL] = Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET;
        absOffset[ISwerveDrive.FR] = Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET;
        absOffset[ISwerveDrive.RL] = Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET;
        absOffset[ISwerveDrive.RR] = Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET;

        turnMotorSim = new FlywheelSim[Constants.NUM_WHEELS];
        turningPIDController = new PIDController[Constants.NUM_WHEELS];
        for(int i=0; i<Constants.NUM_WHEELS; i++) {
            absAngle[i] = absOffset[i];
            //kv = Volt Seconds per Meter
            //ka = ka VoltSecondsSquaredPerMotor
            turnMotorSim[i] = new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.3850, 0.0385),
                DCMotor.getFalcon500(1), 150f/7);
            
            //scale factor for hardware PID to software PID
            //360/2048 is 360 degrees per rev/encoder counts per rev divided by gear ratio
            var k = (360f/2048) * (7f/150);
            turningPIDController[i] = new PIDController(0.4 * k, 0.0005 * k, 0 * k, 0.001);
        }
    }

    @Override
    public void setKinematics(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }
    
    @Override
    public void updateInputs() {
        SwerveModuleState[] swerveStates = new SwerveModuleState[Constants.NUM_WHEELS];

        //xyz_mps[0] = speeds.vxMetersPerSecond / Constants.LOOP_TIME;
        //xyz_mps[1] = speeds.vyMetersPerSecond / Constants.LOOP_TIME;

        //TODO: Simulate the actual swerve corners... https://www.chiefdelphi.com/t/sysid-gains-on-sds-mk4i-modules/400373/7
        for(int i=0; i<swerveStates.length; i++) {
            swerveStates[i] = new SwerveModuleState();

            //process drive command
            if(driveCommand[i] == ControlMode.Velocity) {
                driveSpeed[i] = drivePower[i];
            } else if(driveCommand[i] == ControlMode.PercentOutput) {
                driveSpeed[i] = drivePower[i] * Constants.MAX_DRIVETRAIN_SPEED;
            }else {
                driveSpeed[i] = 0;
            }
            driveDist[i] += driveSpeed[i] * Constants.LOOP_TIME;
            swerveStates[i].speedMetersPerSecond = driveSpeed[i];

            //reset drive command back to zero
            driveCommand[i] = ControlMode.Disabled;
            drivePower[i] = 0;

            //process turn command
            double deltaAngle;
            if(turnCommand[i] == ControlMode.Position) {
                deltaAngle = -turnPower[i];
                for(var loops =0; loops < Constants.LOOP_TIME / 0.001; loops++) {
                    double turnOutput = turningPIDController[i].calculate(turnAngle[i], deltaAngle);
                    //update the sensor values
                    turnAngle[i] += turnOutput;
                    absAngle[i] += turnOutput;
                }
            } else if (turnCommand[i] == ControlMode.PercentOutput) {
                var turnOutput = -turnPower[i] * Constants.NOM_BATTERY_VOLTAGE * Constants.LOOP_TIME / Kv_Turn;
                //update the sensor values
                turnAngle[i] += turnOutput;
                absAngle[i] += turnOutput;
                turnPower[i] = 0;
            } else {
                deltaAngle = 0;
            }
            swerveStates[i].angle = Rotation2d.fromDegrees(turnAngle[i]);

            //reset turn command back to zero
            turnCommand[i] = ControlMode.Disabled;
            turnPower[i] = 0;

            //check to see if we are in scale
            if(Robot.odometry != null) {
                var pose = Robot.odometry.getPose();
                if ((12.09 < pose.getX() && pose.getX() < 12.747) && (1.87 < pose.getY() && pose.getY() < 4.027)) {
                    pitchAngle = 0.12;
                } else if ((11.05 < pose.getX() && pose.getX() < 11.77) && (1.87 < pose.getY() && pose.getY() < 4.027)) {
                    pitchAngle = -0.12;
                } else {
                    pitchAngle = 0;
                }
            }

            /*
            //run at 1ms rate to simulate hardware
            for(var loops =0; loops < Constants.LOOP_TIME / 0.001; loops++) {
                //use the PID to calculate a voltage to apply to the motor
                double turnOutput = turningPIDController[i].calculate(Math.toRadians(absAngle[i]), swerveStates[i].angle.getRadians());
                //check if we exceed battery voltage
                if(turnOutput > Constants.NOM_BATTERY_VOLTAGE) {
                    turnOutput = Constants.NOM_BATTERY_VOLTAGE;
                } else if (turnOutput < -Constants.NOM_BATTERY_VOLTAGE) {
                    turnOutput = -Constants.NOM_BATTERY_VOLTAGE;
                } else {
                    //leave voltage as is
                }
                //set the input
                turnMotorSim[i].setInputVoltage(turnOutput);
                //run it
                turnMotorSim[i].update(0.001);
                //figure out how much angle changed per loop
                double angChange = Math.toDegrees(turnMotorSim[i].getAngularVelocityRadPerSec() * 0.001);
                //update the sensor values
                turnAngle[i] += angChange;
                absAngle[i] += angChange;
            }*/
        }

        ChassisSpeeds speeds = kinematics.toChassisSpeeds(swerveStates);
        chassisAngle += Math.toDegrees(speeds.omegaRadiansPerSecond * Constants.LOOP_TIME);
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(chassisAngle);
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        // Not a Sim function
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        // Not a Sim function
    }

    @Override
    public double getCornerAbsAngle(int wheel) {
        return absAngle[wheel];
    }

    @Override
    public double getCornerAngle(int wheel) {
        return turnAngle[wheel];
    }

    @Override
    public double getCornerSpeed(int wheel) {
        return driveSpeed[wheel];
    }

    @Override
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        driveCommand[wheel] = ControlMode.Velocity;
        drivePower[wheel] = swerveModuleState.speedMetersPerSecond;
        turnCommand[wheel] = ControlMode.Position;
        turnPower[wheel] = swerveModuleState.angle.getDegrees();
    }

    @Override
    public double getCornerDistance(int wheel) {
        return driveDist[wheel];
    }

    @Override
    public void setDriveCommand(int wheel, ControlMode mode, double output) {
        driveCommand[wheel] = mode;
        drivePower[wheel] = output;
    }

    @Override
    public void setTurnCommand(int wheel, ControlMode mode, double output) {
        turnCommand[wheel] = mode;
        turnPower[wheel] = output;
    }

    @Override
    public double getPitch() {
        return pitchAngle;
    }

    @Override
    public double getRoll() {
        // TODO Auto-generated method stub
        return 0;
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
