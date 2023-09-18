package frc.robot.commands;

import org.livoniawarriors.UtilFunctions;

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
        SmartDashboard.putNumber("Turtle Turn Speed", 4);
        drive.SwerveDrive(0, 0, 0, false);
    }

    @Override
    public void execute() {
        boolean fieldOriented = SmartDashboard.getBoolean("Field Oriented", false);
        boostSpeed = UtilFunctions.getSetting(ISwerveDrive.MAX_SPEED_KEY, Constants.MAX_DRIVETRAIN_SPEED);
        turtleSpeed = UtilFunctions.getSetting(ISwerveDrive.TURTLE_SPEED_KEY, 1);
        turtleTurnSpeed = SmartDashboard.getNumber("Turtle Turn Speed", 1);
        double maxSpeed = UtilFunctions.getSetting(ISwerveDrive.MAX_DRIVER_SPEED_KEY, Constants.MAX_DRIVER_SPEED);
        
        double xSpeed = cont.GetXDrivePct();
        double ySpeed = cont.GetYDrivePct();
        double turn   = cont.GetTurnPct();

        double boost = cont.GetBoostTriggerRequest();
        double turtle = cont.GetPrecisionTriggerRequest();

        if(boost > 0.1){
            xSpeed = (xSpeed * maxSpeed) + ((boostSpeed - maxSpeed) * boost * Math.signum(xSpeed));
            ySpeed = (ySpeed * maxSpeed) + ((boostSpeed - maxSpeed) * boost * Math.signum(ySpeed));
            turn = turn * Constants.MAX_DRIVER_OMEGA;
        } else if(turtle > 0.1) {
            xSpeed = (xSpeed * maxSpeed) - ((maxSpeed - turtleSpeed) * turtle * Math.signum(xSpeed));
            ySpeed = (ySpeed * maxSpeed) - ((maxSpeed - turtleSpeed) * turtle * Math.signum(ySpeed));
            turn = turn * turtleTurnSpeed;
        } else {
            xSpeed = xSpeed * maxSpeed;
            ySpeed = ySpeed * maxSpeed;
            turn = turn * Constants.MAX_DRIVER_OMEGA;
        }
        drive.SwerveDrive(xSpeed, ySpeed, turn, fieldOriented);
    }

    @Override
    public boolean isFinished() {
        //never end
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
