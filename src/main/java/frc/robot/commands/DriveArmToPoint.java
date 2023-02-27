package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.interfaces.IOperatorControls;


public class DriveArmToPoint extends CommandBase{
    private Arm arm;
    private IOperatorControls controls;
    private double xPos, zPos;
    private boolean isRunX, isRunZ;    
    public DriveArmToPoint(Arm arm, IOperatorControls controls) {
        this.arm = arm;
        this.controls = controls;
        this.isRunX = false;
        this.isRunZ = false;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        xPos = arm.getArmXPosition();
        zPos = arm.getArmZPosition();
        
        if(controls.GetArmKinXCommand() > 0.5 && !isRunX) {
            xPos += controls.GetArmKinXCommand() + 3;
            isRunX = true;
        } else if(controls.GetArmKinXCommand() < 0.5 && !isRunX) {
            isRunX = false;
        }

        if(controls.GetArmKinZCommand() > 0.5 && !isRunZ) {
            zPos += controls.GetArmKinZCommand() + 3;
            isRunZ = true;
        } else if(controls.GetArmKinZCommand() < 0.5 && !isRunZ) {
            isRunZ = false;
        }        

        if(isRunX || isRunZ) {
            arm.calcAngles(xPos, zPos);
        } else {
            //no command, don't move the arm
        }
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {}
}
