package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.commands.DriveToPoint;
import frc.robot.Odometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public class AutoBalance extends CommandBase {
    private ISwerveDrive drive;
    private Odometry odometry;
    private double pitch;
    private Timer timer;
    
    public AutoBalance(ISwerveDrive drive, Odometry odometry) {
        this.drive = drive;
        this.odometry = odometry;
        this.timer = new Timer();
        timer.reset();
        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this.pitch = drive.getPitch();
        Pose2d currentPose = this.odometry.getPose();
        Pose2d newPose = currentPose;
        //Use pitch to determine if robot is pointing up or down
        //TODO: Modify plus or minus values for new move to location
        if(this.pitch > 1.5) { 
            newPose = new Pose2d(currentPose.getX() + 0.5, currentPose.getY(), null);
        } else if (this.pitch < -1.5) {
            newPose = new Pose2d(currentPose.getX() - 0.5, currentPose.getY(), null);
        } else {
            if(timer.get() == 0) {
                timer.start();
            }
        }
        new DriveToPoint(this.drive, this.odometry, newPose);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3);
    }

    @Override
    public void end(boolean interrupted) {}
}