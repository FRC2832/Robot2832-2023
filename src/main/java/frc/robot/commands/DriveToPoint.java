package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Odometry;
import frc.robot.interfaces.ISwerveDrive;

/**
 * Drive the robot to a specific point on the field 
 */
public class DriveToPoint extends CommandBase {
    private ISwerveDrive drive;
    private Odometry odometry;
    private Pose2d dest;
    private final double TARGET_ERROR = Units.feetToMeters(0.25);
    private boolean isFinished;

    private double distX;
    private double distY;
    private double distLeft;
    private double scale;

    public DriveToPoint(ISwerveDrive drive, Odometry odometry, Pose2d dest) {
        this.drive = drive;
        this.odometry = odometry;
        this.dest = dest;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        isFinished = false;

        //test code!!!  Pick a random point to go to
        Pose2d[] points = {
            new Pose2d(2.25, 5.03, Rotation2d.fromDegrees(-135)),
            new Pose2d(5.97, 10.97, Rotation2d.fromDegrees(45)),
            new Pose2d(2.25, 10.97, Rotation2d.fromDegrees(135)),
            new Pose2d(5.97, 5.03, Rotation2d.fromDegrees(-45)),
        };
        dest = points[(int)(Math.random()*points.length)];
    }

    @Override
    public void execute() {
        Pose2d currentPose = odometry.getPose();
        distX = currentPose.getX() - dest.getX();
        distY = currentPose.getY() - dest.getY();
        distLeft = Math.sqrt((distX * distX) + (distY * distY));

        if(distLeft > TARGET_ERROR) {
            //since we know the dist left, we can scale the speeds based on max distance
            //formula (max speed) / (delta speed) = (distLeft) / (distx/y)
            scale = Constants.MAX_DRIVETRAIN_SPEED / distLeft;
            drive.SwerveDrive(
                distX  * scale, 
                distY  * scale, 
                0, //TODO: Fix Me!
                false);
        } else {
            //we are at our spot, stop
            drive.SwerveDrive(0, 0, 0, false);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("distX", ()-> distX, null);
        builder.addDoubleProperty("distY", ()-> distY, null);
        builder.addDoubleProperty("distLeft", ()-> distLeft, null);
        builder.addDoubleProperty("scale", ()-> scale, null);
    }
}


