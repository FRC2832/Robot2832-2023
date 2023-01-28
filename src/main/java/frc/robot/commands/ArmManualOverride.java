package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.interfaces.IDriveControls;

public class ArmManualOverride extends CommandBase {
    private Arm arm;
    private IDriveControls controls;
    private double elbow;
    private double shoulder;

    public ArmManualOverride(Arm arm, IDriveControls controls){
        this.arm = arm;
        this.controls = controls;
        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shoulder = controls.GetArmShoulderPct() * Constants.NOM_BATTERY_VOLTAGE;
        elbow = controls.GetArmElbowPct() * Constants.NOM_BATTERY_VOLTAGE;
        
        arm.setShoulderMotorVolts(shoulder);
        arm.setElbowMotorVolts(elbow);        
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShoulderMotorVolts(0);
        arm.setElbowMotorVolts(0);
    }
}
