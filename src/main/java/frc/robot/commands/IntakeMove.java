package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Intake;
import frc.robot.interfaces.IOperatorControls;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeMove extends CommandBase{
    private Intake intake;
    private IOperatorControls controls;
    private XboxController operCont;

    public IntakeMove(IOperatorControls controls, Intake intake){
        this.intake = intake;
        this.controls = controls;
        operCont = new XboxController(2);
        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        var sign = -1;
        double intakeVolts = 0.0;
        
        if(controls.IntakeSuckRequested().getAsBoolean()){
            intakeVolts = Constants.IntakeVoltage * operCont.getRightTriggerAxis()* sign;
            intake.setIntakeVolts(intakeVolts);
        }  
        else if(controls.IntakeSpitRequested().getAsBoolean()){
            intakeVolts = -Constants.IntakeVoltage * operCont.getLeftTriggerAxis() * sign;
            intake.setIntakeVolts(intakeVolts);
        } else {
            intake.setIntakeVolts(-1);
        }
        SmartDashboard.putNumber("Intake Voltage", intakeVolts);
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
