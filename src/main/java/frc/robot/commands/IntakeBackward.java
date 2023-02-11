package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GrabberIntake;



public class IntakeBackward extends CommandBase{
    private GrabberIntake intake;

    //need to include the pneumatic code and the methods for that
    public IntakeBackward(GrabberIntake intake) { 
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intake.Grab(false);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeOff();
    }

}
