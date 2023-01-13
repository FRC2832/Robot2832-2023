package frc.robot.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ISwerveDrive extends Subsystem {
    final int FL = 0;
    final int FR = 1;
    final int RL = 2;
    final int RR = 3;

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

    SwerveDriveKinematics getKinematics();
    Rotation2d getHeading();
    SwerveModuleState[] getSwerveStates();
    void setPose(Pose2d robotPose);

    void setTurnMotorBrakeMode(boolean brakeOn);
    void setDriveMotorBrakeMode(boolean brakeOn);
}
