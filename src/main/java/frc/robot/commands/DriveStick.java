package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.interfaces.IDriveControls;
import frc.robot.interfaces.ISwerveDrive;

/**
 * Drive the robot with joysticks 
 */
public class DriveStick extends CommandBase {
    private ISwerveDrive drive;
    private IDriveControls cont;

    /**
     * Inject the drivetain and controller to use
     * @param drive Drivetrain to command
     * @param cont Controller to read from
     */
    public DriveStick(ISwerveDrive drive, IDriveControls cont) {
        this.drive = drive;
        this.cont = cont;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Field Oriented", false);
    }

    @Override
    public void execute() {
        boolean fieldOriented = SmartDashboard.getBoolean("Field Oriented", false);

        double xSpeed = cont.GetXDrivePct();
        double ySpeed = cont.GetYDrivePct();
        double turn   = cont.GetTurnPct();

        drive.SwerveDrive(
            xSpeed  * Constants.MAX_DRIVER_SPEED, 
            ySpeed  * Constants.MAX_DRIVER_SPEED, 
            turn    * Constants.MAX_DRIVER_OMEGA, 
            fieldOriented);
    }

    @Override
    public boolean isFinished() {
        //never end
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
