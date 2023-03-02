package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.Robot;


public class ArmAutonPoint extends CommandBase{
    final static double FLIP_TOLERANCE = 20 / 2;    //tolerance is +/- value, so halve the total distance you want
    private Arm arm;
    private double x, z;
    private double zOrig;
    private double xError, zError, distError;
    private boolean transitionPosToNeg; //add transition from L3 to Tail because that is NOT working currently
    private boolean transitionNegToPos; 
    
    public ArmAutonPoint(Arm arm, double x, double z) {
        this.arm = arm;
        this.x = x;
        zOrig = z;
        transitionPosToNeg = false;
        transitionNegToPos = false;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if(Robot.getGamePieceMode() == Robot.CUBE_MODE){ // if cube mode true: lower z by 4
            z = zOrig - Constants.ScoreCubeOffset;
        } else {
            z = zOrig;
        }
        if(Math.signum(arm.getArmXPosition()) == 1 && Math.signum(x) == -1){
            transitionPosToNeg = true;
        } else if(Math.signum(arm.getArmXPosition()) == -1 && Math.signum(x) == 1){
            transitionNegToPos = true;
        } else {
            transitionPosToNeg = false;
            transitionNegToPos = false;
        }

        arm.resetPids();
    }

    @Override
    public void execute() {
        if(transitionPosToNeg){
            arm.calcAngles(Constants.ArmToTransitionPoint_X, Constants.ArmToTransitionPoint_Z);
            if (arm.getArmXPosition() > Constants.ArmToTransitionPoint_X - FLIP_TOLERANCE && arm.getArmXPosition() < Constants.ArmToTransitionPoint_X + FLIP_TOLERANCE){
                transitionPosToNeg = false;
            }
        } else if(transitionNegToPos){
            arm.calcAngles(Constants.ArmToTransitionPoint2_X, Constants.ArmToTransitionPoint2_Z);
            if (arm.getArmXPosition() > Constants.ArmToTransitionPoint2_X - FLIP_TOLERANCE && arm.getArmXPosition() < Constants.ArmToTransitionPoint2_X + FLIP_TOLERANCE){
                transitionNegToPos = false;
            }
        } else{
            arm.calcAngles(x, z);
        }
    }

    @Override
    public boolean isFinished() {
        xError = Math.abs(x - arm.getArmXPosition());
        zError = Math.abs(z - arm.getArmZPosition());
        distError = Math.sqrt((xError * xError) + (zError * zError));
        boolean fin = (distError < Constants.ARM_ACCEPT_ERROR);
        return fin; 
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShoulderMotorVolts(0);
        arm.setElbowMotorVolts(0);
    }
}
