package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.interfaces.IOperatorControls;

public class ArmManualOverride extends CommandBase {
    private Arm arm;
    private IOperatorControls controls;
    private double elbow;
    private double shoulder;

    public ArmManualOverride(Arm arm, IOperatorControls controls){
        this.arm = arm;
        this.controls = controls;
        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shoulder = controls.GetArmShoulderPct() * RobotController.getBatteryVoltage();
        elbow = controls.GetArmElbowPct() * RobotController.getBatteryVoltage();
        
        // if(arm.getArmXPosition() < 0){
        //     shoulder = -1 * shoulder;
        //     elbow = -1 * elbow;
        // }

        arm.setShoulderMotorVolts(shoulder);
        arm.setElbowMotorVolts(elbow);
        arm.resetPids();
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
