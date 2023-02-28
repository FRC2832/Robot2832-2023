package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GrabberIntake;

public class IntakeForward extends CommandBase{
    
    private GrabberIntake intake;
    private int finishedCounts;

    public IntakeForward(GrabberIntake intake) { 
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        finishedCounts = 0;
    }

    @Override
    public void execute() {
        intake.Grab(true);
        finishedCounts++;
    }

    @Override
    public boolean isFinished() {
        if(finishedCounts > 50){
            return true;
        }
        else {
            return false; 
        } 
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeOff();
    }
}