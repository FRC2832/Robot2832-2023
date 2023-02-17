package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.IDriveControls;
import frc.robot.interfaces.ISwerveDrive;

public class Odometry extends SubsystemBase {
    private final boolean PLOT_SWERVE_CORNERS = false;
    SwerveDriveOdometry odometry;
    ISwerveDrive drive;
    IDriveControls controls;
    Pose2d robotPose = new Pose2d();
    private final Field2d field = new Field2d();
    private Translation2d[] swervePositions;

    public Odometry(ISwerveDrive drive, IDriveControls controls) {
        super();
        this.drive = drive;
        this.controls = controls;

        swervePositions = drive.getCornerLocations();
        odometry = new SwerveDriveOdometry(drive.getKinematics(), drive.getHeading(), drive.getSwerveStates());
        SmartDashboard.putData("Field", field);
    }
    
    @Override
    public void periodic() {
        Rotation2d heading = drive.getHeading();
        SwerveModulePosition[] states = drive.getSwerveStates();
        robotPose = odometry.update(heading, states);
        drive.setPose(robotPose);
        field.setRobotPose(robotPose);

        if(controls.IsFieldOrientedResetRequested()) {
            resetHeading();
        }

        if(PLOT_SWERVE_CORNERS) {
            // Update the poses for the swerveModules. Note that the order of rotating the
            // position and then adding the translation matters
            var modulePoses = new Pose2d[swervePositions.length];
            for (int i = 0; i < swervePositions.length; i++) {
                Translation2d modulePositionFromChassis = swervePositions[i].rotateBy(heading).plus(robotPose.getTranslation());

                // Module's heading is it's angle relative to the chassis heading
                modulePoses[i] = new Pose2d(modulePositionFromChassis,
                    states[i].angle.plus(robotPose.getRotation()));
            }
            field.getObject("Swerve Modules").setPoses(modulePoses);
        }
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(drive.getHeading(), drive.getSwerveStates(), pose);
    }

    public void resetHeading() {
        //reset the robot back to it's spot, just facing forward now
        Pose2d pose = new Pose2d(robotPose.getTranslation(),Rotation2d.fromDegrees(0));
        odometry.resetPosition(drive.getHeading(), drive.getSwerveStates(), pose);
    }

    public Pose2d getPose() {
        return robotPose;
    }
}
