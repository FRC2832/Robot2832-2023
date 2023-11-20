package frc.robot.commands;

import frc.robot.Pivot;
import edu.wpi.first.wpilibj.DriverStation;
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
        double presentPivotAngle = pivot.getPivotAngle();
        if(Math.abs(presentPivotAngle) > 360){
            angleDeg += (int)(presentPivotAngle/360)*360.0;
        }
    }

    @Override
    public void execute() { 
        pivot.setPivotAngle(angleDeg);
    }


    @Override
    public boolean isFinished() {
        if(DriverStation.isTeleop()) {
            return false;
        }
        return Math.abs(pivot.getPivotAngle() - angleDeg) < 3;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setPivotMotorVolts(0);
    }
}