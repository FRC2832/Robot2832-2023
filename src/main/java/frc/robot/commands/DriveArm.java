package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
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
        arm.setElbowMotorVolts(controls.GetArmElbowPct() * RobotController.getBatteryVoltage());
        arm.setShoulderMotorVolts(controls.GetArmShoulderPct() * RobotController.getBatteryVoltage());
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {}
}
