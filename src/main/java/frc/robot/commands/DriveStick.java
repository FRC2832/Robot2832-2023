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
    private double boostSpeed;
    private double turtleSpeed;
    private double turtleTurnSpeed;

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
        SmartDashboard.putNumber("Boost Speed", Constants.MAX_DRIVETRAIN_SPEED);
        SmartDashboard.putNumber("Turtle Speed", 1);
        SmartDashboard.putNumber("Turtle Turn Speed", 4);
    }

    @Override
    public void execute() {
        boolean fieldOriented = SmartDashboard.getBoolean("Field Oriented", false);
        boostSpeed = SmartDashboard.getNumber("Boost Speed", Constants.MAX_DRIVETRAIN_SPEED);
        turtleSpeed = SmartDashboard.getNumber("Turtle Speed", 1);
        turtleTurnSpeed = SmartDashboard.getNumber("Turtle Turn Speed", 4);
        double xSpeed = cont.GetXDrivePct();
        double ySpeed = cont.GetYDrivePct();
        double turn   = cont.GetTurnPct();
        if(cont.BoostTriggerRequested()){
            drive.SwerveDrive(
                (xSpeed + cont.GetPercentRightTriggerAxis())  * boostSpeed, //maybe make max driver speed constants into boost constants? or get boost constants from a driver profile?
                (ySpeed + cont.GetPercentRightTriggerAxis())  * boostSpeed, 
                turn    * Constants.MAX_DRIVER_OMEGA, 
                fieldOriented);
        } else if(cont.PrecisionTriggerRequested()) {
            drive.SwerveDrive(
                (xSpeed + cont.GetPercentLeftTriggerAxis())  * turtleSpeed, 
                (ySpeed + cont.GetPercentLeftTriggerAxis())  * turtleSpeed, 
                turn    * turtleTurnSpeed, 
                fieldOriented);
        } 
    }

    @Override
    public boolean isFinished() {
        //never end
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
