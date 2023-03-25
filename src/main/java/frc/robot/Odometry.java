package frc.robot;

import java.io.IOException;
import java.security.interfaces.RSAKey;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.interfaces.IDriveControls;
import frc.robot.interfaces.ISwerveDrive;

public class Odometry extends SubsystemBase {
    private PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private final boolean PLOT_SWERVE_CORNERS = false;
    SwerveDriveOdometry odometry;
    ISwerveDrive drive;
    IDriveControls controls;
    Pose2d robotPose = new Pose2d();
    private final Field2d field = new Field2d();
    private Translation2d[] swervePositions;

    //base arm simulation
    private Mechanism2d m_mech2d;
    private MechanismLigament2d shoulderBar;
    private MechanismLigament2d elbowBar;
    private MechanismLigament2d tailBar;
    private Arm arm;
    private Tail tail;

    public Odometry(ISwerveDrive drive, IDriveControls controls, Arm arm, Tail tail) {
        super();
        this.drive = drive;
        this.controls = controls;
        this.tail = tail;
        this.arm = arm;

        swervePositions = drive.getCornerLocations();
        odometry = new SwerveDriveOdometry(drive.getKinematics(), drive.getHeading(), drive.getSwerveStates());
        SmartDashboard.putData("Field", field);

        camera = new PhotonCamera("USB_Camera");
        try {
            aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch(IOException e) {
            // TODO decide what you want to do if the layout fails to load
        }

        initArm();
    }

    public void setDriverControls(IDriveControls controls) {
        this.controls = controls;
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

        //update the arm
        shoulderBar.setAngle(Rotation2d.fromDegrees(arm.getShoulderAngle()));
        elbowBar.setAngle(Rotation2d.fromDegrees(arm.getElbowAngle()));
        tailBar.setAngle(Rotation2d.fromDegrees(tail.getTailAngle()));

        //check the camera
        var result = camera.getLatestResult().getBestTarget();
        if(result != null) {
            Transform3d target = result.getBestCameraToTarget();
            var angle = Math.toDegrees(target.getRotation().getAngle());

            //var newPose = target4.transformBy(new Transform2d(new Translation2d(target.getX(),target.getY()), Rotation2d.fromDegrees(angle)));
            //resetPose(newPose);
            SmartDashboard.putNumber("Target X", target.getX());
            SmartDashboard.putNumber("Target Y", target.getY());
            SmartDashboard.putNumber("Target Z", target.getZ());
            SmartDashboard.putNumber("Target ID", result.getFiducialId());
            SmartDashboard.putNumber("Target Angle", angle);

            if(camera.getLatestResult().hasTargets()) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target, aprilTagFieldLayout.getTagPose(result.getFiducialId()).get(), new Transform3d());
                resetPose(robotPose.toPose2d());
            }
        }
    }

    Pose2d target4 = new Pose2d(1.487, 2.998, Rotation2d.fromDegrees(0));

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

    private void initArm() {
        //all units in inches, 0* angle is straight right
        //ligaments are used a lot because they will be shown in the picture, roots are not
        m_mech2d = new Mechanism2d(120, 90);

        //build the field object
        MechanismRoot2d midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0);
        midNodeHome.append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
        MechanismRoot2d highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0);
        highNodeHome.append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
        MechanismRoot2d gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0);
        gridHome.append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));

        //build the robot
        MechanismRoot2d robotBase = m_mech2d.getRoot("Robot Frame", 49.75, 3.5); //edge of grid wall, middle of bumpers now
        robotBase.append(new MechanismLigament2d("Bumpers", 38, 0, 50, new Color8Bit(Color.kDarkGreen)));    //32" robot frame + 3" for bumpers each side
        MechanismRoot2d pivotBase = m_mech2d.getRoot("Pivot Base", 56.25, 16.5);  //edge of grid wall + 3" bumper + 3.5", ~5" frame height + 11.5" height piece
        pivotBase.append(new MechanismLigament2d("Arm Frame", 13, 270, 15, new Color8Bit(Color.kSilver)));
        shoulderBar = new MechanismLigament2d("Shoulder", 36, 60, 15, new Color8Bit(Color.kPurple));
        pivotBase.append(shoulderBar);
        elbowBar = new MechanismLigament2d("Elbow", 28, -60, 15, new Color8Bit(Color.kGold));
        shoulderBar.append(elbowBar);

        //tail
        MechanismRoot2d tailBase = m_mech2d.getRoot("Tail Base", 84.25, 10.5);
        tailBase.append(new MechanismLigament2d("Tail Frame", 7, 270, 15, new Color8Bit(Color.kSilver)));
        tailBar = new MechanismLigament2d("Tail", 18, -105, 15, new Color8Bit(Color.kWhite));
        tailBase.append(tailBar);

        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Arm Sim", m_mech2d);
    }
}
