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
        xPos = 36;
        zPos = 36;
    }

    @Override
    public void execute() {
        //take the current position and +/- 0.1" per loop
        xPos += controls.GetArmKinXCommand() * 0.2;
        zPos += -controls.GetArmKinZCommand() * 0.2;
        var x = xPos;
        var z = zPos;

        double shoulder = 0;
        double elbow = 0;
        if(x >= 0) {
            double l = x;
            double h = Math.sqrt(l * l + z * z);
            double phi = Math.toDegrees(Math.atan(z/l));
            double theta = Math.toDegrees(Math.acos((h/2)/30));
            shoulder = phi + theta;
            elbow = (phi - theta);
        }
        else{
            double l = -x;
            double h = Math.sqrt(l * l + z * z);
            double phi = Math.toDegrees(Math.atan(z/l));
            double theta = Math.toDegrees(Math.acos((h/2)/30));
            shoulder = 180 - (phi + theta);
            elbow = 180 - (phi - theta);
        }
        arm.setElbowAngle(elbow);
        arm.setShoulderAngle(shoulder);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {}
}
