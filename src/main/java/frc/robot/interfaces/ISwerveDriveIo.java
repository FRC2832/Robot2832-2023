package frc.robot.interfaces;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public  interface ISwerveDriveIo {
    final String FL_OFFSET_KEY = "Swerve Hardware/FL Offset";
    final String FR_OFFSET_KEY = "Swerve Hardware/FR Offset";
    final String RL_OFFSET_KEY = "Swerve Hardware/RL Offset";
    final String RR_OFFSET_KEY = "Swerve Hardware/RR Offset";

    void updateInputs();
    Rotation2d getHeading();
    double getPitch();
    double getRoll();
    Translation2d[] getCornerLocations();
    double getWheelOffset(int wheel);
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

    /**
     * Returns the distance the swerve corner as traveled in meters
     * @param wheel Which corner to look at
     * @return The distance the swerve corner as traveled in meters
     */
    double getCornerDistance(int wheel);
    void setCornerState(int wheel, SwerveModuleState swerveModuleState);

    void setDriveCommand(int wheel, ControlMode mode, double output);

    void setTurnCommand(int wheel, ControlMode mode, double output);
}
