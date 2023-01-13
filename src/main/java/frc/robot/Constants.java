package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;

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
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 12;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 22;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 32;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 42;

    //CanCoder CAN ids for swerve corners
    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 13;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 23;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 33;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 43;

    //CanCoder zero locations
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = 310.7;
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = 318.0;
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = 179.2;
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = 19.33;

    //Swerve corner locations for kinematics
    //24 3/8" for the distance between 2 wheels + 2 1.5" wheel widths = 11.4375" off the center point of the robot
    public static final Translation2d SWERVE_FRONT_LEFT_LOCATION = new Translation2d(0.291, 0.291);
    public static final Translation2d SWERVE_FRONT_RIGHT_LOCATION = new Translation2d(0.291, -0.291);
    public static final Translation2d SWERVE_BACK_LEFT_LOCATION = new Translation2d(-0.291, 0.291);
    public static final Translation2d SWERVE_BACK_RIGHT_LOCATION = new Translation2d(-0.291, -0.291);

    public static final GearRatio SWERVE_GEAR_SET = Mk4iSwerveModuleHelper.GearRatio.L2;  //16.3 ft/s
    public static final double MAX_DRIVETRAIN_SPEED = 4.96;         //max meters per second the swerve modules can go
    public static final double MAX_DRIVETRAIN_OMEGA = 3 * Math.PI;  //max Radians per Second the robot can spin
    public static final double NOM_BATTERY_VOLTAGE = 12.5;

    public static final double MAX_DRIVER_SPEED = 3;                //Max speed (meters/sed) the driver can go
    public static final double MAX_DRIVER_OMEGA = 1.5 * Math.PI;    //Max angle (rad/sec) the driver can go
    public static final double STICK_DEADBAND = 0.13;               //how much of the sticks from the driver should we remove

    public static final Pose2d START_POS = new Pose2d(3.186,6.072,Rotation2d.fromDegrees(-135));  //where does the robot start at?
}
