package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.GrabberIntake;
import frc.robot.Robot;
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
        var sign = 1;
        if(Robot.getGamePieceMode() == Robot.CUBE_MODE) {
            sign = -1;
        }
        
        if(controls.GrabberSuckRequested().getAsBoolean()){
            grabber.setIntakeVolts(-Constants.IntakeVoltage * sign);
        }  
        else if(controls.GrabberSpitRequested().getAsBoolean()){
            grabber.setIntakeVolts(Constants.IntakeVoltage * sign);
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
        grabber.setIntakeVolts(0);
    }
}
