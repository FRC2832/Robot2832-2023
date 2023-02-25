package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.Robot;


public class ArmAutonPoint extends CommandBase{
    private Arm arm;
    private double x, z;
    private double zOrig;
    private double xError, zError;
    private boolean transition; //add transition from L3 to Tail because that is NOT working currently

    
    public ArmAutonPoint(Arm arm, double x, double z) {
        this.arm = arm;
        this.x = x;
        zOrig = z;
        transition = false;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if(Robot.getGamePieceMode() == Robot.CUBE_MODE){ // if cube mode true: lower z by 4
            z = zOrig - 4;
        } else {
            z = zOrig;
        }
        if(Math.signum(x) != Math.signum(arm.getArmXPosition())){
            transition = true;
        }
    }

    @Override
    public void execute() {
        if(transition){
            arm.calcAngles(Constants.ArmToTransitionPoint_X, Constants.ArmToTransitionPoint_Z);
            if (arm.getArmXPosition() > Constants.ArmToTransitionPoint_X - 3 && arm.getArmXPosition() < Constants.ArmToTransitionPoint_X + 3){
                transition = false;
            }
        } else {
            arm.calcAngles(x, z);
        }
    }

    @Override
    public boolean isFinished() {
        xError = Math.abs(x - arm.getArmXPosition());
        zError = Math.abs(z - arm.getArmZPosition());
        boolean fin = (xError < Constants.ARM_ACCEPT_ERROR) && (zError < Constants.ARM_ACCEPT_ERROR);
        return fin; 
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShoulderMotorVolts(0);
        arm.setElbowMotorVolts(0);
    }
}
