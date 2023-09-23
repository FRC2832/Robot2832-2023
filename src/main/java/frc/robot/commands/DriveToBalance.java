package frc.robot.commands;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.interfaces.ISwerveDrive;

/**
 * Drive the robot until we hit the scale
 */
public class DriveToBalance extends CommandBase {
    private ISwerveDrive drive;
    public int finishedCounts;
    private double balanceConst;

    /**
     * Drive the robot up the rest of the ramp and balance it.  Only works in forward direction!
     * @param drive Drivetrain subsystem to command
     * @param stopTime Time to drive
     */
    public DriveToBalance(ISwerveDrive drive) {
        //copy inputs to the command
        this.drive = drive;

        //add this command to the drivetrain subsystem
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        finishedCounts = 0;

        System.out.println("Balance Start");
        if(DriverStation.isFMSAttached()) {
            //bigger = slower
            //28 for our scale, 32 for kettering, 34 for official scale
            balanceConst = UtilFunctions.getSetting("DriveToBalance/FmsBalanceConst", 37);
        } else {
            //we are on our field, go faster            
            balanceConst = UtilFunctions.getSetting("DriveToBalance/HomeBalanceConst", 35);
        }
    }

    @Override
    public void execute() {
        var pitch = drive.getPitch();

        if(Math.abs(pitch) < 2) {
            finishedCounts++;
            drive.SwerveDrive(0, 0, 0, false);
        } else {
            finishedCounts = 0;
            
            var speed = Constants.MAX_AUTO_SPEED * Math.abs(pitch) / balanceConst;    //this number is the tuning constant, larger number = slower the robot drives (ideal pitch up the ramp is 12*)
            var speed2 = -Math.signum(pitch) * Math.max(speed, Constants.MIN_DRIVER_SPEED);
            SmartDashboard.putNumber("Balance Speed", speed2);

            drive.SwerveDrive(speed2, 0, 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        //check if the timer has finished
        return finishedCounts > 25;
    }

    @Override
    public void end(boolean interrupted) {
        //when stopped, stop the drivetrain
        drive.SwerveDrive(0, 0, 0, false);
    }
}
