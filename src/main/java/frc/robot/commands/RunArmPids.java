package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RunArmPids extends CommandBase {
    public RunArmPids() {
        addRequirements();
    }

    @Override
    public void initialize() {
        Robot.setRunArmPidsAllTime(!Robot.getRunArmPidsAllTime());
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
