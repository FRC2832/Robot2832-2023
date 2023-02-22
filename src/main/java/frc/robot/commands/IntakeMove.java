package frc.robot.commands;

import frc.robot.Intake;
import frc.robot.interfaces.IDriveControls;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakeMove extends CommandBase{
    private Intake intake;
    private IDriveControls controls;
    private double angleOffset;

    public IntakeMove(IDriveControls controls, Intake intake){
        this.intake = intake;
        this.controls = controls;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        angleOffset = 0;
    }


    @Override
    public void execute() {  
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
