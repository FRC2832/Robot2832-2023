package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArmPids extends CommandBase {
    private static boolean run = false; 

    public RunArmPids() {
        addRequirements();
    }

    @Override
    public void initialize() {
        RunArmPids.run = !RunArmPids.run;
    }

    @Override
    public void execute() {}

    public boolean getRun() {
        return RunArmPids.run;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
