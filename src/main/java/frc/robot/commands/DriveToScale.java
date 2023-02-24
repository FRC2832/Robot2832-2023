package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.interfaces.ISwerveDrive;

/**
 * Drive the robot until we hit the scale
 */
public class DriveToScale extends CommandBase {
    private ISwerveDrive drive;
    public int finishedCounts;

    /**
     * Drive the robot at 50% power for stopTime time.
     * @param drive Drivetrain subsystem to command
     * @param stopTime Time to drive
     */
    public DriveToScale(ISwerveDrive drive) {
        //copy inputs to the command
        this.drive = drive;

        //add this command to the drivetrain subsystem
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        finishedCounts = 0;
    }

    @Override
    public void execute() {
        drive.SwerveDrive(Constants.MAX_AUTO_SPEED, 0, 0, false);

        if(drive.getPitch() < -8) {
            finishedCounts++;
        } else {
            finishedCounts = 0;
        }
    }

    @Override
    public boolean isFinished() {
        //check if the timer has finished
        return finishedCounts > 50;
    }

    @Override
    public void end(boolean interrupted) {
        //when stopped, stop the drivetrain
        drive.SwerveDrive(0, 0, 0, false);
    }
}
