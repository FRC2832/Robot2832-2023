package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Intake;
import frc.robot.Robot;

public class IntakeForward extends CommandBase{
    private int finishedCounts;
    private Intake intake;

    public IntakeForward(Intake intake) { 
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        finishedCounts = 0;
        intake.resetTimer();
    }

    @Override
    public void execute() {
        intake.Grab(true);
        finishedCounts++;
    }

    @Override
    public boolean isFinished() {
        if(finishedCounts < 10){
            return false;
        } else if (finishedCounts > 100) {
            return true;
        }
        if(intake.HasPiece()){
            return false;
        }
        else {
            return true; 
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeOff();
    }
}