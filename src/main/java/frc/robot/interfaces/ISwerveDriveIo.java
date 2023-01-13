package frc.robot.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public  interface ISwerveDriveIo {
    void updateInputs();
    Rotation2d getHeading();
    void setKinematics(SwerveDriveKinematics kinematics);
    void setTurnMotorBrakeMode(boolean brakeOn);
    void setDriveMotorBrakeMode(boolean brakeOn);

    /**
     * Returns the CANcoder absolute angle of the swerve corner in degrees
     * @param wheel Which corner to look at
     * @return The absolute angle of the swerve corner in degrees
     */
    double getCornerAbsAngle(int wheel);
    /**
     * Returns the turn motor absolute angle of the swerve corner in degrees
     * @param wheel Which corner to look at
     * @return The absolute angle of the swerve corner in degrees
     */
    double getCornerAngle(int wheel);
    /**
     * Returns the speed of the swerve corner in meters per second
     * @param wheel Which corner to look at
     * @return The speed of the swerve corner in meters per second
     */
    double getCornerSpeed(int wheel);
    void setCornerState(int wheel, SwerveModuleState swerveModuleState);
}
