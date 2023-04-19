package frc.robot.commands;

import frc.robot.Intake;
import frc.robot.Pivot;
import frc.robot.Robot;
import frc.robot.interfaces.IOperatorControls;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class PivotMove extends CommandBase{
    private Pivot pivot;
    private Intake intake;
    private IOperatorControls controls;
    private double angleOffset;
    private double targetAngle;
    private boolean pieceMode;

    public PivotMove(IOperatorControls controls, Intake intake, Pivot pivot){
        this.pivot = pivot;
        this.intake = intake;
        this.controls = controls;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        angleOffset = -pivot.getPivotAngle() + pivot.optimalPivotAngle();
        pieceMode = Robot.getGamePieceMode();
    }

    private boolean lastPiece = false;
    @Override
    public void execute() {  
        //reset the offset angle when the piece mode changed
        boolean newMode = Robot.getGamePieceMode();
        boolean hasPiece = intake.HasPiece();
        if(pieceMode != newMode) { //|| (hasPiece == false && lastPiece == true)) {
            angleOffset = 0;
            pivot.resetRotations();
        }
        lastPiece = hasPiece;
        pieceMode = newMode;

        if(controls.IntakeUpRequested().getAsBoolean()){
            pivot.setPivotMotorVolts(7);
            angleOffset = pivot.optimalPivotAngle() - pivot.getPivotAngle();
            targetAngle = pivot.getPivotAngle();
        } else if(controls.IntakeDownRequested().getAsBoolean()){
            pivot.setPivotMotorVolts(-7);
            angleOffset = pivot.optimalPivotAngle() - pivot.getPivotAngle();
            targetAngle = pivot.getPivotAngle();
        } else {
            pivot.setPivotAngle(targetAngle);
            //pivot.setPivotMotorVolts(0);
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
