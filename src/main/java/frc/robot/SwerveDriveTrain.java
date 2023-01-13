package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.interfaces.ISwerveDriveIo;

public class SwerveDriveTrain implements ISwerveDrive {
    private SwerveDriveKinematics kinematics;
    private ISwerveDriveIo hardware;
    private SwerveModuleState[] swerveStates;
    private Pose2d robotPose;
    private String moduleNames[];
    private double swerveOffsets[];
    private double turnOffsets[];

    public SwerveDriveTrain(ISwerveDriveIo hSwerveDriveIo) {
        this.hardware = hSwerveDriveIo;

        //initialize the corner locations
        kinematics = new SwerveDriveKinematics(
            Constants.SWERVE_FRONT_LEFT_LOCATION,
            Constants.SWERVE_FRONT_RIGHT_LOCATION,
            Constants.SWERVE_BACK_LEFT_LOCATION,
            Constants.SWERVE_BACK_RIGHT_LOCATION);
        hardware.setKinematics(kinematics);
        
        //initialize the swerve states
        swerveStates = new SwerveModuleState[Constants.NUM_WHEELS];
        for(int wheel = 0; wheel < Constants.NUM_WHEELS; wheel++) {
            swerveStates[wheel] = new SwerveModuleState();
        }

        //initialize the swerve offsets
        swerveOffsets = new double[Constants.NUM_WHEELS];
        swerveOffsets[FL] = Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET;
        swerveOffsets[FR] = Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET;
        swerveOffsets[RL] = Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET;
        swerveOffsets[RR] = Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET;

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

        //initialize module names
        moduleNames = new String[Constants.NUM_WHEELS];
        moduleNames[FL] = "Module FL/";
        moduleNames[FR] = "Module FR/";
        moduleNames[RL] = "Module RL/";
        moduleNames[RR] = "Module RR/";
    }
    
    @Override
    public void periodic() {
        hardware.updateInputs();

        //read the swerve corner state
        for(int wheel = 0; wheel < Constants.NUM_WHEELS; wheel++) {
            swerveStates[wheel].speedMetersPerSecond = hardware.getCornerSpeed(wheel);
            //double angle = (hardware.getCornerAbsAngle(wheel) - swerveOffsets[wheel]) + (hardware.getCornerAngle(wheel) - turnOffsets[wheel]);
            double angle = (hardware.getCornerAngle(wheel) - turnOffsets[wheel]);
            swerveStates[wheel].angle = Rotation2d.fromDegrees(angle);
        }

        //display data on SmartDashboard
        SmartDashboard.putNumber("Gyro Angle", getHeading().getDegrees());
        for(int wheel=0; wheel < Constants.NUM_WHEELS; wheel++) {
            SmartDashboard.putNumber(moduleNames[wheel] + "Abs Sensor", hardware.getCornerAbsAngle(wheel));
            SmartDashboard.putNumber(moduleNames[wheel] + "Turn Sensor", hardware.getCornerAngle(wheel));
            SmartDashboard.putNumber(moduleNames[wheel] + "Drive Speed Sensor", hardware.getCornerSpeed(wheel));
            SmartDashboard.putNumber(moduleNames[wheel] + "Calc Angle", swerveStates[wheel].angle.getDegrees());
            
            SmartDashboard.putNumber(moduleNames[wheel] + "ABS Offset", swerveOffsets[wheel]);
            SmartDashboard.putNumber(moduleNames[wheel] + "Turn Offset", turnOffsets[wheel]);
        }
    }

    @Override
    public void SwerveDrive(double xSpeed, double ySpeed, double turn, boolean fieldOriented) {
        // ask the kinematics to determine our swerve command
        ChassisSpeeds speeds;
        if (fieldOriented) {
            //90* is needed since we view the field on a 90* rotation
            var angle = robotPose.getRotation().minus(Rotation2d.fromDegrees(90));
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
            //figure out delta angle from the current swerve state
            var delta = requestStates[i].angle.minus(swerveStates[i].angle);
            //add it to the current hardware motor angle since we control that motor
            requestStates[i].angle = Rotation2d.fromDegrees(hardware.getCornerAngle(i)).plus(delta);
            hardware.setCornerState(i, requestStates[i]);

            SmartDashboard.putNumber(moduleNames[i] + "Command Angle", requestStates[i].angle.getDegrees());
            SmartDashboard.putNumber(moduleNames[i] + "Command Speed", requestStates[i].speedMetersPerSecond);
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
    public SwerveModuleState[] getSwerveStates() {
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
}
