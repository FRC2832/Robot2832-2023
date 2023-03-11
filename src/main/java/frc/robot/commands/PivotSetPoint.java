package frc.robot.commands;

import frc.robot.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotSetPoint extends CommandBase{
    private Pivot pivot;
    private double angleDeg;
    
    public PivotSetPoint(Pivot pivot, double angleDeg){
        this.pivot = pivot;
        this.angleDeg = angleDeg;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() { 
        pivot.setPivotAngle(angleDeg);
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
