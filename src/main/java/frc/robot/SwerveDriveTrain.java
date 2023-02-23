package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.interfaces.ISwerveDriveIo;

public class SwerveDriveTrain implements ISwerveDrive {
    private SwerveDriveKinematics kinematics;
    private ISwerveDriveIo hardware;
    private SwerveModulePosition[] swerveStates;
    private Pose2d robotPose;
    private String moduleNames[];
    private double swerveOffsets[];
    private double turnOffsets[];
    private InterpolatingTreeMap<Double, Double> speedReduction;
    private NetworkTable currentTable;
    private NetworkTable tempTable;
    private double gyroOffset = 0;
    private PIDController pidZero = new PIDController(0.1, 0.001, 0);

    public SwerveDriveTrain(ISwerveDriveIo hSwerveDriveIo) {
        this.hardware = hSwerveDriveIo;

        currentTable = NetworkTableInstance.getDefault().getTable("/motor_currents");
        tempTable = NetworkTableInstance.getDefault().getTable("/motor_temps");

        //initialize the corner locations
        kinematics = new SwerveDriveKinematics(hSwerveDriveIo.getCornerLocations());
        hardware.setKinematics(kinematics);
        
        //initialize the swerve states
        swerveStates = new SwerveModulePosition[Constants.NUM_WHEELS];
        for(int wheel = 0; wheel < Constants.NUM_WHEELS; wheel++) {
            swerveStates[wheel] = new SwerveModulePosition();
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
        moduleNames[FL] = "Module FL/";
        moduleNames[FR] = "Module FR/";
        moduleNames[RL] = "Module RL/";
        moduleNames[RR] = "Module RR/";

        //input is angle off desired, output is percent reduction
        speedReduction = new InterpolatingTreeMap<Double, Double>();
        speedReduction.put(0., 1.);
        speedReduction.put(45., 1.);
        speedReduction.put(90., 1.0);
    }
    
    @Override
    public void periodic() {
        hardware.updateInputs();

        //read the swerve corner state
        for(int wheel = 0; wheel < Constants.NUM_WHEELS; wheel++) {
            swerveStates[wheel].distanceMeters = hardware.getCornerDistance(wheel);
            //double angle = (hardware.getCornerAbsAngle(wheel) - swerveOffsets[wheel]) + (hardware.getCornerAngle(wheel) - turnOffsets[wheel]);
            double angle = (hardware.getCornerAngle(wheel) - turnOffsets[wheel]);
            swerveStates[wheel].angle = Rotation2d.fromDegrees(angle);
        }

        //display data on SmartDashboard
        SmartDashboard.putNumber("Gyro Angle", getHeading().getDegrees());       
        SmartDashboard.putNumber("Pitch", hardware.getPitch());
        SmartDashboard.putNumber("Roll", hardware.getRoll());
        for(int wheel=0; wheel < Constants.NUM_WHEELS; wheel++) {
            SmartDashboard.putNumber(moduleNames[wheel] + "Abs Sensor", hardware.getCornerAbsAngle(wheel));
            SmartDashboard.putNumber(moduleNames[wheel] + "Turn Sensor", hardware.getCornerAngle(wheel));
            SmartDashboard.putNumber(moduleNames[wheel] + "Drive Speed Sensor", hardware.getCornerSpeed(wheel));
            SmartDashboard.putNumber(moduleNames[wheel] + "Calc Angle", swerveStates[wheel].angle.getDegrees());
            
            SmartDashboard.putNumber(moduleNames[wheel] + "ABS Offset", swerveOffsets[wheel]);
            SmartDashboard.putNumber(moduleNames[wheel] + "Turn Offset", turnOffsets[wheel]);

            tempTable.getEntry(moduleNames[wheel] + "Drive").setDouble(hardware.getDriveMotorTemperature(wheel));
            tempTable.getEntry(moduleNames[wheel] + "Turn").setDouble(hardware.getTurnMotorTemperature(wheel));
            currentTable.getEntry(moduleNames[wheel] + "Drive").setDouble(hardware.getDriveMotorCurrent(wheel));
            currentTable.getEntry(moduleNames[wheel] + "Turn").setDouble(hardware.getTurnMotorCurrent(wheel));
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
                //figure out delta angle from the current swerve state
                var delta = requestStates[i].angle.minus(swerveStates[i].angle);
                
                //reduce drive speed based on how far the wheel angle is off
                var reduction = speedReduction.get(Math.abs(delta.getDegrees()));
                requestStates[i].speedMetersPerSecond *= reduction;

                //add it to the current hardware motor angle since we control that motor
                requestStates[i].angle = Rotation2d.fromDegrees(hardware.getCornerAngle(i)).plus(delta).times(-1);
                hardware.setCornerState(i, requestStates[i]);
            }

            SmartDashboard.putNumber(moduleNames[i] + "Command Angle", requestStates[i].angle.getDegrees());
            SmartDashboard.putNumber(moduleNames[i] + "Command Speed", requestStates[i].speedMetersPerSecond);
        }
    }

    public void setWheelCommand(SwerveModuleState[] requests) {
        for(int i=0; i<requests.length; i++) {
            //figure out delta angle from the current swerve state
            var delta = requests[i].angle.minus(swerveStates[i].angle);
            //add it to the current hardware motor angle since we control that motor
            requests[i].angle = Rotation2d.fromDegrees(hardware.getCornerAngle(i)).plus(delta).times(-1);
            hardware.setCornerState(i, requests[i]);
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
