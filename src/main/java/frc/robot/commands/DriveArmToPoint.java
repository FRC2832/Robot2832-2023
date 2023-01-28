package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.interfaces.IDriveControls;


public class DriveArmToPoint extends CommandBase{
    private Arm arm;
    private IDriveControls controls;
    private double xPos, zPos;
    
    public DriveArmToPoint(Arm arm, IDriveControls controls) {
        this.arm = arm;
        this.controls = controls;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        xPos = arm.getArmXPosition();
        zPos = arm.getArmZPosition();
        //take the current position and +/- max 0.2" per loop
        xPos += controls.GetArmKinXCommand() * 0.2;
        zPos += -controls.GetArmKinZCommand() * 0.2;
        var x = xPos;
        var z = zPos;

        arm.calcAngles(x, z);
    }

    @Override
    public boolean isFinished() {
        return true; 
    }

    @Override
    public void end(boolean interrupted) {}
}
