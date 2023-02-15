package frc.robot.commands;
import frc.robot.GrabberIntake;
import frc.robot.interfaces.IDriveControls;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class GrabberMove extends CommandBase{
    private GrabberIntake grabber;
    private IDriveControls controls;

    public GrabberMove(IDriveControls controls, GrabberIntake grabber){
        this.grabber = grabber;
        this.controls = controls;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {}


    @Override
    public void execute() {  
        if(controls.GrabberSuckRequested().getAsBoolean()){
            grabber.setIntakeVolts(-4);// TODO: Determine angle of "UP" position
        }  
        if(controls.GrabberSpitRequested().getAsBoolean()){
            grabber.setIntakeVolts(4); // TODO: Determine angle of "DOWN" position
        }  
    }


    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void end(boolean interrupted) {
        grabber.setIntakeVolts(0);
    }
}
