package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.interfaces.IDriveControls;


public class moveArmToAngles extends CommandBase{
    private Arm arm;
    private IDriveControls controls;
    private double xPos, zPos;
    private double shoulderAng, elbowAng;
    
    public moveArmToAngles(Arm arm, double shoulderAng, double elbowAng) {
        this.arm = arm;
        this.shoulderAng = shoulderAng;
        this.elbowAng = elbowAng;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        arm.setElbowAngle(elbowAng);
        arm.setShoulderAngle(shoulderAng);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {}
}
