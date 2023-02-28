package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Intake;
import frc.robot.Robot;
import frc.robot.interfaces.IOperatorControls;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakeMove extends CommandBase{
    private Intake intake;
    private IOperatorControls controls;

    public IntakeMove(IOperatorControls controls, Intake intake){
        this.intake = intake;
        this.controls = controls;
        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        var sign = 1;
        if(Robot.getGamePieceMode() == Robot.CUBE_MODE) {
            sign = -1;
        }
        
        if(controls.IntakeSuckRequested().getAsBoolean()){
            intake.setIntakeVolts(-Constants.IntakeVoltage * sign);
        }  
        else if(controls.IntakeSpitRequested().getAsBoolean()){
            intake.setIntakeVolts(Constants.IntakeVoltage * sign);
        } else {
            intake.setIntakeVolts(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeVolts(0);
    }
}
