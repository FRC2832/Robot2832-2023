package frc.robot.commands;

import frc.robot.Pivot;
import frc.robot.Robot;
import frc.robot.interfaces.IOperatorControls;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class PivotMove extends CommandBase{
    private Pivot pivot;
    private IOperatorControls controls;
    private double angleOffset;
    private boolean pieceMode;

    public PivotMove(IOperatorControls controls, Pivot pivot){
        this.pivot = pivot;
        this.controls = controls;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        angleOffset = -pivot.getPivotAngle() + pivot.optimalPivotAngle();
        pieceMode = Robot.getGamePieceMode();
    }

    @Override
    public void execute() {  
        //reset the offset angle when the piece mode changed
        boolean newMode = Robot.getGamePieceMode();
        if(pieceMode != newMode) {
            angleOffset = 0;
            pivot.resetRotations();
        }
        pieceMode = newMode;

        if(controls.IntakeUpRequested().getAsBoolean()){
            pivot.setPivotMotorVolts(5);
            angleOffset = pivot.optimalPivotAngle() - pivot.getPivotAngle();
        } else if(controls.IntakeDownRequested().getAsBoolean()){
            pivot.setPivotMotorVolts(-5);
            angleOffset = pivot.optimalPivotAngle() - pivot.getPivotAngle();
        } else {
            pivot.setPivotAngle(pivot.optimalPivotAngle() - angleOffset);
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
