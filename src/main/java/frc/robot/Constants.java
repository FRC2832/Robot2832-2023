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
    //to change, zero the wheel to flat, then read the calc angle value.  Subtract that to the offset.
    //like if old offset is 119, and the new zeroed offset is 35, the new value is 84
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = 142.2;
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = 13.8;
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = 227.5;
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = 84.0;

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
    public static final double MAX_DRIVER_SPEED = 3.3;                //Max speed (meters/sec) the driver can go
    public static final double MAX_DRIVER_OMEGA = 1.8 * Math.PI;    //Max angle (rad/sec) the driver can go
    public static final double STICK_DEADBAND = 0.13;               //how much of the sticks from the driver should we remove
    public static final double MAX_AUTO_SPEED = 1.5;//2.5;                //Max speed (meters/sec) the robot can go in auton
    public static final double MAX_AUTO_TURN_SPEED = 3 * Math.PI;                //Max speed (rad/sec) the robot can rotate
    
    public static final Pose2d START_BLUE_LEFT = new Pose2d(2.5,4.75,Rotation2d.fromDegrees(0));  //starting positions for Auton
    public static final Pose2d START_BLUE_MIDDLE = new Pose2d(2.5,2.75,Rotation2d.fromDegrees(0)); 
    public static final Pose2d START_BLUE_RIGHT = new Pose2d(2.5,1.15,Rotation2d.fromDegrees(0)); 
    public static final Pose2d START_BLUE_THREE_PIECE = new Pose2d(2.5,4.2,Rotation2d.fromDegrees(0)); 
    public static final Pose2d START_RED_LEFT = new Pose2d(13.5,4.75,Rotation2d.fromDegrees(180)); 
    public static final Pose2d START_RED_MIDDLE = new Pose2d(13.5,2.75,Rotation2d.fromDegrees(180)); 
    public static final Pose2d START_RED_RIGHT = new Pose2d(13.5,1.15,Rotation2d.fromDegrees(180)); 
    public static final Pose2d START_RED_THREE_PIECE = new Pose2d(13.5,4.2,Rotation2d.fromDegrees(180)); 
    
    public static final double BICEP_LENGTH = 32.0;         //length in inches of the bicep on the arm mechanism
    public static final double FOREARM_LENGTH = 23.5;       //length in inches of the forearm on the arm mechanism

    //sensor offsets
    public static final double SHOULDER_OFFSET = 14;       //90 is shoulder straight up
    public static final double ELBOW_OFFSET = 195;         //0 is elbow parallel to shoulder
    public static final double INTAKE_OFFSET = 78-25;

    public static final double ARM_ACCEPT_ERROR = 3;      //MAX error in inches for arm going to set points autonomously 
    
    //Arm set point constants 
    //pickup ground front
    public static final double ArmToPickupGround_X = 45.6;
    public static final double ArmToPickupGround_Z = -0.5;

    //pickup from tail
    public static final double ArmToPickupTail_X = 25.2;
    public static final double ArmToPickupTail_Z = 16.0;
    public static final double PivotToPickupTail = 150;

    //pickup from human player station
    public static final double ArmToPickupHuman_X = 26.6;
    public static final double ArmToPickupHuman_Z = 8.0;
    public static final double PivotToPickupHuman = 162;

    //level 2 back
    public static final double ArmToScoreMiddle_X = -22.5;
    public static final double ArmToScoreMiddle_Z = 26.3;
    //223 pivot

    //level 2 front
    public static final double ArmToScoreMiddleFront_X = 50.0;
    public static final double ArmToScoreMiddleFront_Z = 23;
    public static final double PivotToMiddleFront = 41;
    //558 pivot

    //level 3 back
    public static final double ArmToScoreTop_X = -39;
    public static final double ArmToScoreTop_Z = 38.5;
    //575 pivot

    //level 3 auton //X = -24.2 Z = 30.0
    public static final double ArmToScoreTopAuto_X = -24.2;
    public static final double ArmToScoreTopAuto_Z = 37.0;

////////////////////////////////////////////////////////////////////////
    //level 1 back
    public static final double ArmToScoreLow_X = -26.6;
    public static final double ArmToScoreLow_Z = 6;

    //stow arm inside frame
    public static final double ArmToSecureLocation_X = 20;
    public static final double ArmToSecureLocation_Z = 0;

    //pickup ground back (only used in auto)
    public static final double ArmToPickupGroundBack_X = -20;
    public static final double ArmToPickupGroundBack_Z = 2;

    public static final double ScoreCubeOffset = 0;

    public static final double IntakeVoltage = 12.5;
    public static final double TAIL_LOW_POINT = 0;
    public static final double TAIL_HIGH_POINT = 105;
    public static final double TAIL_STOW_POINT = 150;
    public static final double TAIL_HUMAN_POINT = 52;
    
    
    public static final double CTRE_P_RES = 1024.0 / 12;
}
