package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.interfaces.IOperatorControls;

public class DriveArm extends CommandBase{
    private Arm arm;
    private IOperatorControls controls;
    
    public DriveArm(Arm arm, IOperatorControls controls) {
        this.arm = arm;
        this.controls = controls;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        arm.setElbowMotorVolts(controls.GetArmElbowPct() * Constants.NOM_BATTERY_VOLTAGE);
        arm.setShoulderMotorVolts(controls.GetArmShoulderPct() * Constants.NOM_BATTERY_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {}
}
