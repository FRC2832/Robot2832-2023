package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
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
    private double distRot;
    private double distLeft;
    private double scale;
    private double scaleRot;

    public DriveToPoint(ISwerveDrive drive, Odometry odometry, Pose2d dest) {
        this.drive = drive;
        this.odometry = odometry;
        this.dest = dest;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = odometry.getPose();
        distX = dest.getX() - currentPose.getX();
        distY = dest.getY() - currentPose.getY();
        distRot = dest.getRotation().getDegrees() - (currentPose.getRotation().getDegrees());
        distLeft = Math.sqrt((distX * distX) + (distY * distY));

        if(distLeft > TARGET_ERROR) {
            //since we know the dist left, we can scale the speeds based on max distance
            //formula (max speed) / (delta speed) = (distLeft) / (distx/y)
            scale = Constants.MAX_AUTO_SPEED / distLeft;
            scaleRot = Constants.MAX_AUTO_TURN_SPEED / (distLeft * 2);
            drive.SwerveDrive(
                distX  * scale, 
                distY  * scale, 
                distRot * scaleRot, //TODO: Fix Me! might be fixed?
                true);
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


