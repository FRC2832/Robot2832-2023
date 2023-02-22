package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static final int NUM_WHEELS = 4;

    public static final int PIGEON_IMU_ID = 50;
    public static final double LOOP_TIME = 0.02;

    //falcon 500 CAN ids for Drive Motors
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 11;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 21;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 31;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 41;

    //falcon 500 CAN ids for Turn Motors
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 12;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 22;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 32;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 42;

    //CanCoder CAN ids for swerve corners
    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 13;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 23;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 33;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 43;

    //CanCoder zero locations
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = 142.2;
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = 10.8;
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = 90.5;
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = 120.4;

    //Swerve corner locations for kinematics
    //22.25 -1.5" width 10.375"
    //26.75" for the distance , 13.375"
    public static final Translation2d SWERVE_FRONT_LEFT_LOCATION = new Translation2d(0.264, 0.340);
    public static final Translation2d SWERVE_FRONT_RIGHT_LOCATION = new Translation2d(0.264, -0.340);
    public static final Translation2d SWERVE_BACK_LEFT_LOCATION = new Translation2d(-0.264, 0.340);
    public static final Translation2d SWERVE_BACK_RIGHT_LOCATION = new Translation2d(-0.264, -0.340);

    public static final double MAX_DRIVETRAIN_SPEED = 4.96;         //max meters per second the swerve modules can go
    public static final double MAX_DRIVETRAIN_OMEGA = 3 * Math.PI;  //max Radians per Second the robot can spin
    public static final double NOM_BATTERY_VOLTAGE = 12.5;

    public static final double MIN_DRIVER_SPEED = 0.3;                  //Min speed (meters/sec) used in SwerveDriveTrain
    public static final double MAX_DRIVER_SPEED = 1.5;                //Max speed (meters/sed) the driver can go
    public static final double MAX_DRIVER_OMEGA = 1.0 * Math.PI;    //Max angle (rad/sec) the driver can go
    public static final double STICK_DEADBAND = 0.13;               //how much of the sticks from the driver should we remove

    public static final Pose2d START_BLUE_LEFT = new Pose2d(2.5,4.75,Rotation2d.fromDegrees(0));  //starting positions for Auton
    public static final Pose2d START_BLUE_MIDDLE = new Pose2d(2.5,2.75,Rotation2d.fromDegrees(0)); 
    public static final Pose2d START_BLUE_RIGHT = new Pose2d(2.5,1.15,Rotation2d.fromDegrees(0)); 
    public static final Pose2d START_RED_LEFT = new Pose2d(13.5,4.75,Rotation2d.fromDegrees(180)); 
    public static final Pose2d START_RED_MIDDLE = new Pose2d(13.5,2.75,Rotation2d.fromDegrees(180)); 
    public static final Pose2d START_RED_RIGHT = new Pose2d(13.5,1.15,Rotation2d.fromDegrees(180)); 
    
    public static final double BICEP_LENGTH = 34.0;         //length in inches of the bicep on the arm mechanism
    public static final double FOREARM_LENGTH = 26.0;       //length in inches of the forearm on the arm mechanism

    //sensor offsets
    public static final double SHOULDER_OFFSET = 347;
    public static final double ELBOW_OFFSET = 135;
    public static final double INTAKE_OFFSET = -41;

    public static final double ARM_ACCEPT_ERROR = 0.5;      //MAX error in inches for arm going to set points autonomously 
    
    //Arm set point constants 
    public static final double ArmToPickupGround_X = 36;
    public static final double ArmToPickupGround_Z = -2;

    public static final double ArmToPickupTail_X = 21;
    public static final double ArmToPickupTail_Z = 14.6;

    public static final double ArmToPickupHuman_X = 30;
    public static final double ArmToPickupHuman_Z = 20;

    public static final double ArmToSecureLocation_X = 20;
    public static final double ArmToSecureLocation_Z = 0;

    public static final double ArmToScoreLow_X = -29;
    public static final double ArmToScoreLow_Z = 2;

    public static final double ArmToScoreMiddle_X = -25.5;
    public static final double ArmToScoreMiddle_Z = 30;

    public static final double ArmToScoreTop_X = -40;
    public static final double ArmToScoreTop_Z = 42;

    public static final double IntakeVoltage = 12.5;
    
}
