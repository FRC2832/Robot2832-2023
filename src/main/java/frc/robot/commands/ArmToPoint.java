package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.Robot;


public class ArmToPoint extends CommandBase{
    private Arm arm;
    
    public ArmToPoint(Arm arm, double x, double z) {
        this.arm = arm;
        addRequirements(arm);
        SmartDashboard.putNumber("Arm Pos X", 36);
        SmartDashboard.putNumber("Arm Pos Z", 36);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        var x = SmartDashboard.getNumber("Arm Pos X", 36);
        var z = SmartDashboard.getNumber("Arm Pos Z", 36);

        if(Robot.getGamePieceMode()){ // if cube mode true: lower z by 4
            z -= 4;
        }

        arm.calcAngles(x, z);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {}
}
