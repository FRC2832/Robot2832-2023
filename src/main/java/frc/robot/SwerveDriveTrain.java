package frc.robot;

import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.interfaces.ISwerveDriveIo;

public class SwerveDriveTrain implements ISwerveDrive {
    private SwerveDriveKinematics kinematics;
    private ISwerveDriveIo hardware;
    private SwerveModulePosition[] swerveStates;
    private SwerveModuleState[] swerveTargets;
    private Pose2d robotPose;
    private String moduleNames[];
    private double swerveOffsets[];
    private double turnOffsets[];
    private InterpolatingTreeMap<Double, Double> speedReduction;
    private double gyroOffset = 0;
    private PIDController pidZero = new PIDController(0.15, 0.001, 0);
    private PIDController[] drivePid;
    private PIDController[] turnPid;

    public SwerveDriveTrain(ISwerveDriveIo hSwerveDriveIo) {
        register();
        this.hardware = hSwerveDriveIo;

        //initialize the corner locations
        kinematics = new SwerveDriveKinematics(hSwerveDriveIo.getCornerLocations());
        hardware.setKinematics(kinematics);
        
        //initialize the swerve states
        swerveStates = new SwerveModulePosition[Constants.NUM_WHEELS];
        swerveTargets = new SwerveModuleState[Constants.NUM_WHEELS];
        drivePid = new PIDController[Constants.NUM_WHEELS];
        turnPid = new PIDController[Constants.NUM_WHEELS];
        for(int wheel = 0; wheel < Constants.NUM_WHEELS; wheel++) {
            swerveStates[wheel] = new SwerveModulePosition();
            swerveTargets[wheel] = new SwerveModuleState();
            drivePid[wheel] = new PIDController(0.5, 0.03, 0.0);
            turnPid[wheel] = new PIDController(5,1.8,0);
        }

        //initialize the swerve offsets
        swerveOffsets = new double[Constants.NUM_WHEELS];
        swerveOffsets[FL] = hardware.getWheelOffset(FL);
        swerveOffsets[FR] = hardware.getWheelOffset(FR);
        swerveOffsets[RL] = hardware.getWheelOffset(RL);
        swerveOffsets[RR] = hardware.getWheelOffset(RR);

        hardware.updateInputs();
        turnOffsets = new double[Constants.NUM_WHEELS];
        for(int i=0; i<turnOffsets.length; i++) {
            double offset = (swerveOffsets[i] - hardware.getCornerAbsAngle(i));
            if (offset > 180) {
                offset -= 360;
            } else if (offset < -180) {
                offset += 360;
            }
            turnOffsets[i] = offset + hardware.getCornerAngle(i);
        }
        gyroOffset = getHeading().getDegrees();

        //initialize module names
        moduleNames = new String[Constants.NUM_WHEELS];
        moduleNames[FL] = "Swerve FL ";
        moduleNames[FR] = "Swerve FR ";
        moduleNames[RL] = "Swerve RL ";
        moduleNames[RR] = "Swerve RR ";

        //input is angle off desired, output is percent reduction
        speedReduction = new InterpolatingTreeMap<Double, Double>();
        speedReduction.put(0., 1.);
        speedReduction.put(45., 1.);
        speedReduction.put(90., 1.);
    }
    
    @Override
    public void periodic() {
        hardware.updateInputs();
        SwerveModuleState[] currentState = new SwerveModuleState[Constants.NUM_WHEELS];

        //read the swerve corner state
        for(int wheel = 0; wheel < Constants.NUM_WHEELS; wheel++) {
            swerveStates[wheel].distanceMeters = hardware.getCornerDistance(wheel);

            double angle = hardware.getCornerAbsAngle(wheel) - swerveOffsets[wheel];
            angle = MathUtil.inputModulus(angle, -180, 180);
            swerveStates[wheel].angle = Rotation2d.fromDegrees(angle);

            currentState[wheel] = new SwerveModuleState();
            currentState[wheel].angle = swerveStates[wheel].angle;
            currentState[wheel].speedMetersPerSecond = hardware.getCornerSpeed(wheel);

            if(DriverStation.isDisabled()) {
                turnPid[wheel].reset();
            }
        }

        Logger.PushSwerveStates(currentState,swerveTargets);
        //display data on SmartDashboard
        for(int wheel=0; wheel < Constants.NUM_WHEELS; wheel++) {
            SmartDashboard.putNumber(moduleNames[wheel] + "Calc Angle", swerveStates[wheel].angle.getDegrees());
        }
    }

    @Override
    public void SwerveDrive(double xSpeed, double ySpeed, double turn, boolean fieldOriented) {
        // ask the kinematics to determine our swerve command
        ChassisSpeeds speeds;

        double currentHeading = getHeading().getDegrees();
        if (Math.abs(turn) > 0.1) {
            //if a turn is requested, reset the zero for the drivetrain
            gyroOffset = currentHeading;
            pidZero.reset();
        } else {
            //straighten the robot
            turn = pidZero.calculate(currentHeading,gyroOffset);
        }

        if (fieldOriented) {
            var angle = robotPose.getRotation();
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turn, angle);
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, turn);
        }
        
        //calculate the states from the speeds
        SwerveModuleState[] requestStates = kinematics.toSwerveModuleStates(speeds);
        // sometime the Kinematics spits out too fast of speeds, so this will fix this
        SwerveDriveKinematics.desaturateWheelSpeeds(requestStates, Constants.MAX_DRIVETRAIN_SPEED);

        // command each swerve module
        for (int i = 0; i < requestStates.length; i++) {
            SmartDashboard.putNumber(moduleNames[i] + "Requested Angle", requestStates[i].angle.getDegrees());
            
            //check to see if the robot request is moving
            if (Math.abs(requestStates[i].speedMetersPerSecond) < Constants.MIN_DRIVER_SPEED) {
                //stop the requests if there is no movement
                hardware.setDriveCommand(i, ControlMode.Disabled, 0);
                hardware.setTurnCommand(i, ControlMode.Disabled, 0.0);
            }
            else {
                requestStates[i] = SwerveModuleState.optimize(requestStates[i], swerveStates[i].angle);
                
                var driveVolts = drivePid[i].calculate(hardware.getCornerSpeed(i),requestStates[i].speedMetersPerSecond);
                var ff = requestStates[i].speedMetersPerSecond / Constants.MAX_DRIVETRAIN_SPEED * RobotController.getBatteryVoltage();
                driveVolts += ff;

                var turnVolts = -turnPid[i].calculate(swerveStates[i].angle.getRadians(),requestStates[i].angle.getRadians());
                
                hardware.setDriveCommand(i, ControlMode.PercentOutput, driveVolts / RobotController.getBatteryVoltage());
                hardware.setTurnCommand(i, ControlMode.PercentOutput, turnVolts / RobotController.getBatteryVoltage());
            }
            swerveTargets = requestStates;

            SmartDashboard.putNumber(moduleNames[i] + "Command Angle", requestStates[i].angle.getDegrees());
            SmartDashboard.putNumber(moduleNames[i] + "Command Speed", requestStates[i].speedMetersPerSecond);
        }
    }

    public void setWheelCommand(SwerveModuleState[] requestStates) {
        swerveTargets = requestStates;
        for(int i=0; i<requestStates.length; i++) {
            requestStates[i] = SwerveModuleState.optimize(requestStates[i], swerveStates[i].angle);
                
            var driveVolts = drivePid[i].calculate(hardware.getCornerSpeed(i),requestStates[i].speedMetersPerSecond);
            var ff = requestStates[i].speedMetersPerSecond / Constants.MAX_DRIVETRAIN_SPEED * RobotController.getBatteryVoltage();
            driveVolts += ff;

            var turnVolts = -turnPid[i].calculate(swerveStates[i].angle.getRadians(),requestStates[i].angle.getRadians());
            
            hardware.setDriveCommand(i, ControlMode.PercentOutput, driveVolts / RobotController.getBatteryVoltage());
            hardware.setTurnCommand(i, ControlMode.PercentOutput, turnVolts / RobotController.getBatteryVoltage());
        }
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public Rotation2d getHeading() {
        return hardware.getHeading();
    }

    @Override
    public double getPitch() {
        return hardware.getPitch();
    }

    @Override
    public SwerveModulePosition[] getSwerveStates() {
        return swerveStates;
    }

    @Override
    public void setPose(Pose2d robotPose) {
        this.robotPose = robotPose;        
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        hardware.setTurnMotorBrakeMode(brakeOn);
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        hardware.setDriveMotorBrakeMode(brakeOn);
    }

    @Override
    public Translation2d[] getCornerLocations() {
        return hardware.getCornerLocations();
    }
}
