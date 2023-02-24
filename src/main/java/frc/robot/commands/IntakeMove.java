package frc.robot.commands;

import frc.robot.Intake;
import frc.robot.Robot;
import frc.robot.interfaces.IOperatorControls;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakeMove extends CommandBase{
    private Intake intake;
    private IOperatorControls controls;
    private double angleOffset;
    private boolean pieceMode;

    public IntakeMove(IOperatorControls controls, Intake intake){
        this.intake = intake;
        this.controls = controls;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        angleOffset = 0;
        intake.resetRotations();
        pieceMode = Robot.getGamePieceMode();
    }


    @Override
    public void execute() {  
        //reset the offset angle when the piece mode changed
        boolean newMode = Robot.getGamePieceMode();
        if(pieceMode != newMode) {
            angleOffset = 0;
            intake.resetRotations();
        }
        pieceMode = newMode;

        if(controls.GrabberUpRequested().getAsBoolean()){
            intake.setPivotMotorVolts(6);
            angleOffset = intake.optimalIntakeAngle() - intake.getPivotAngle();
        } else if(controls.GrabberDownRequested().getAsBoolean()){
            intake.setPivotMotorVolts(-6);
            angleOffset = intake.optimalIntakeAngle() - intake.getPivotAngle();
        } else {
            intake.setPivotAngle(intake.optimalIntakeAngle() - angleOffset);
        }
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPivotMotorVolts(0);
    }
}
