package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.interfaces.IOperatorControls;


public class DriveArmToPoint extends CommandBase{
    private Arm arm;
    private IOperatorControls controls;
    private double xPos, zPos;
    private boolean running;
 
    public DriveArmToPoint(Arm arm, IOperatorControls controls) {
        this.arm = arm;
        this.controls = controls;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        xPos = arm.getArmXPosition();
        zPos = arm.getArmZPosition();
        arm.resetPids();
        running = false;
    }

    @Override
    public void execute() {
            if(Math.abs(controls.GetArmKinXCommand()) > 0.1 || Math.abs(controls.GetArmKinZCommand()) > 0.1) {
                xPos += controls.GetArmKinXCommand()*.3;
                zPos += controls.GetArmKinZCommand()*-.3;
                running = true;
            } else if(running == true) {
                xPos = arm.getArmXPosition();
                zPos = arm.getArmZPosition();
                arm.resetPids();
                running = false;
            } else {
                //hold position we were at
            }
            
            arm.calcAngles(xPos, zPos);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
