package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.GrabberIntake;
import frc.robot.interfaces.IOperatorControls;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class GrabberMove extends CommandBase{
    private GrabberIntake grabber;
    private IOperatorControls controls;

    public GrabberMove(IOperatorControls controls, GrabberIntake grabber){
        this.grabber = grabber;
        this.controls = controls;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {}


    @Override
    public void execute() {  
        if(controls.GrabberSuckRequested().getAsBoolean()){
            grabber.setIntakeVolts(-Constants.IntakeVoltage);// TODO: Determine angle of "UP" position
        }  
        else if(controls.GrabberSpitRequested().getAsBoolean()){
            grabber.setIntakeVolts(Constants.IntakeVoltage); // TODO: Determine angle of "DOWN" position
        } else {
            grabber.setIntakeVolts(0);
        }
    }


    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void end(boolean interrupted) {
        grabber.setIntakeVolts(Constants.IntakeVoltage);
    }
}
