package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArmPids extends CommandBase {
    private boolean run; 

    public RunArmPids() {
        this.run = false;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        this.run = !this.run;
    }

    public boolean getRun() {
        return this.run;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
