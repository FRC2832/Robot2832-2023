package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class ChangeMode extends CommandBase{
    private boolean currentMode;
    private String mode;
    
    public ChangeMode() {
        addRequirements();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        currentMode = Robot.getGamePieceMode();
        Robot.setGamePieceMode(!currentMode);
        if(currentMode)
            mode = "Cone!";
        else
            mode = "Cube!";
        SmartDashboard.putString("Piece Mode", mode);
    }

    @Override
    public boolean isFinished() {
        return true; 
    }

    @Override
    public void end(boolean interrupted) {}
}
