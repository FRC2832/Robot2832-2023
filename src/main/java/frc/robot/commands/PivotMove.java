package frc.robot.commands;

import frc.robot.Intake;
import frc.robot.Pivot;
import frc.robot.interfaces.IOperatorControls;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotMove extends CommandBase{
    private Pivot pivot;
    private IOperatorControls controls;
    private double targetAngle;

    public PivotMove(IOperatorControls controls, Intake intake, Pivot pivot){
        this.pivot = pivot;
        this.controls = controls;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {  
        if(controls.IntakeUpRequested().getAsBoolean()){
            pivot.setPivotMotorVolts(7);
            targetAngle = pivot.getPivotAngle();
        } else if(controls.IntakeDownRequested().getAsBoolean()){
            pivot.setPivotMotorVolts(-7);
            targetAngle = pivot.getPivotAngle();
        } else {
            pivot.setPivotAngle(targetAngle);
        }
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setPivotMotorVolts(0);
    }
}
