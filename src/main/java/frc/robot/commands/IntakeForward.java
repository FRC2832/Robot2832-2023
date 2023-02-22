package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GrabberIntake;



public class IntakeForward extends CommandBase{
    
    private GrabberIntake intake;
    private Timer timer;

    public IntakeForward(GrabberIntake intake) { 
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.resetTimer();
    }

    @Override
    public void execute() {
        intake.Grab(true);
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