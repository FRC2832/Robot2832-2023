package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Intake;
import frc.robot.Robot;

public class IntakeBackward extends CommandBase{
    private int finishedCounts;
    private Intake intake;

    public IntakeBackward(Intake intake) { 
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
        intake.Grab(false);
        finishedCounts++;
    }

    @Override
    public boolean isFinished() {
        if(finishedCounts < 10){
            return false;
        }
        if(intake.HasPiece() || finishedCounts > 100){
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
