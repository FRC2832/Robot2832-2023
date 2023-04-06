package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ChangeMode extends CommandBase{
    private boolean currentMode;
    private String mode;

    public ChangeMode() {
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        currentMode = Robot.getGamePieceMode();
        Robot.setGamePieceMode(!currentMode);
        if(currentMode)
            mode = "Cone!";
        else
            mode = "Cube!";
        SmartDashboard.putString("Piece Mode", mode);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
