package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.interfaces.IDriveControls;


public class ArmPointToPointDifLengths extends CommandBase{
    

    private Arm armLen;
    private IDriveControls controls;
    private double xPos, zPos;
    
    public ArmPointToPointDifLengths(Arm armLen, IDriveControls controls) {
        this.armLen = armLen;
        this.controls = controls;
        addRequirements(armLen);
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
        //elbow sometimes shows NaN, sometimes shows 0.0, not anything that i hard code it to
        double elbow = 0;
        //40-45 finds elbow angle in radians, but havent tested to see which angle is for the up reaching arm and down reaching arm
        if(x >= 0) {
            elbow = 3.14159 - Math.acos((Constants.BICEP_LENGTH*Constants.BICEP_LENGTH + Constants.FOREARM_LENGTH*Constants.FOREARM_LENGTH - x*x - z*z)/(2*Constants.BICEP_LENGTH*Constants.FOREARM_LENGTH));
        }
        else{
            elbow = Math.acos((x*x + z*z - Constants.BICEP_LENGTH*Constants.BICEP_LENGTH - Constants.FOREARM_LENGTH*Constants.FOREARM_LENGTH)/(2*Constants.BICEP_LENGTH*Constants.FOREARM_LENGTH));
        }
        // finds shoulder angle in radians
        shoulder = Math.atan(z/x) - Math.atan((Constants.FOREARM_LENGTH*Math.sin(elbow))/(Constants.BICEP_LENGTH + Constants.FOREARM_LENGTH*Math.cos(elbow)));
        elbow = Math.toDegrees(elbow);
        shoulder = Math.toDegrees(shoulder);


        armLen.setElbowAngle(elbow);
        armLen.setShoulderAngle(shoulder);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {}
}
