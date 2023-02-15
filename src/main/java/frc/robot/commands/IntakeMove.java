package frc.robot.commands;
import frc.robot.GrabberIntake;
import frc.robot.Intake;
import frc.robot.Tail;
import frc.robot.TailHw;
import frc.robot.interfaces.IDriveControls;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakeMove extends CommandBase{
    private Intake intake;
    private IDriveControls controls;

    public IntakeMove(IDriveControls controls, Intake intake){
        this.intake = intake;
        this.controls = controls;
        addRequirements(intake);
    }

    @Override
    public void initialize() {}


    @Override
    public void execute() {  
        if(controls.GrabberUpRequested().getAsBoolean()){
            intake.setPivotMotorVolts(3.5); // TODO: Determine angle of "UP" position
        }  
        if(controls.GrabberDownRequested().getAsBoolean()){
            intake.setPivotMotorVolts(-3.5); // TODO: Determine angle of "DOWN" position
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
