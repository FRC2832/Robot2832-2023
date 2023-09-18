package frc.robot.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ISwerveDrive extends Subsystem {
    final int FL = 0;
    final int FR = 1;
    final int RL = 2;
    final int RR = 3;

    /** The fastest rate we want the drive wheels to change speeds in m/s */
    final String MAX_ACCEL_KEY = "Swerve Drive/Max Wheel Accel";
    /** The fastest rate we want the swerve wheels to turn */
    final String MAX_OMEGA_KEY = "Swerve Drive/Max Wheel Omega";
    /** How fast we want the driver to go during normal operation */
    final String MAX_DRIVER_SPEED_KEY = "Swerve Drive/Max Driver Speed";
    /** The max speed possible with the swerve wheels */
    final String MAX_SPEED_KEY = "Swerve Drive/Max Speed";
    /** How slow we want the driver to go in turtle mode */
    final String TURTLE_SPEED_KEY = "Swerve Drive/Turtle Speed";
    /** The angle we want the swerve to invert the request to get to position faster */
    final String OPTOMIZE_ANGLE_KEY = "Swerve Drive/Optomize Angle";

     /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    void SwerveDrive(double xSpeed, double ySpeed, double turn, boolean fieldOriented);

    Translation2d[] getCornerLocations();
    SwerveDriveKinematics getKinematics();
    Rotation2d getHeading();
    double getPitch();
    
    SwerveModulePosition[] getSwerveStates();
    void setPose(Pose2d robotPose);

    void setTurnMotorBrakeMode(boolean brakeOn);
    void setDriveMotorBrakeMode(boolean brakeOn);
    void setWheelCommand(SwerveModuleState[] requests);
}
