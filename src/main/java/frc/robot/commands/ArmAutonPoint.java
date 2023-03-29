package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.Robot;


public class ArmAutonPoint extends CommandBase{
    final static double FLIP_TOLERANCE = 6.0; //Constants.ARM_ACCEPT_ERROR;
    private Arm arm;
    private double x, z;
    private double zOrig;
    private double xError, zError, distError;
    private boolean transitionPosToNeg; //add transition from L3 to Tail because that is NOT working currently
    private boolean transitionNegToPos; 
    private boolean moveElbow1;
    private boolean moveElbow2;
    
    public ArmAutonPoint(Arm arm, double x, double z) {
        this.arm = arm;
        this.x = x;
        zOrig = z;
        transitionPosToNeg = false;
        transitionNegToPos = false;
        moveElbow1 = false;
        moveElbow2 = false;
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
                moveElbow1 = true; 
                transitionPosToNeg = false;
            } 
        } else if(transitionNegToPos){
            arm.calcAngles(Constants.ArmToTransitionPoint2_X, Constants.ArmToTransitionPoint2_Z);
            if (arm.getArmXPosition() > Constants.ArmToTransitionPoint2_X - FLIP_TOLERANCE && arm.getArmXPosition() < Constants.ArmToTransitionPoint2_X + FLIP_TOLERANCE){
            //if (FLIP_TOLERANCE > Math.sqrt(Math.pow(arm.getArmXPosition() - Constants.ArmToTransitionPoint2_X, 2) + Math.pow(arm.getArmZPosition() - Constants.ArmToTransitionPoint2_Z, 2))){
                moveElbow2 = true; 
                transitionNegToPos = false;
            }
        } else if (moveElbow1) {
            arm.setElbowAngle(90);
            if (Math.abs(arm.getElbowAngle() - 90) < 7){
                moveElbow1 = false;
            }
        } else if (moveElbow2) {
            arm.setElbowAngle(-90);
            if (Math.abs(arm.getElbowAngle() + 90) < 7){
                moveElbow2 = false;
            }
        } else { 
            arm.calcAngles(x, z);
        }
    }

    @Override
    public boolean isFinished() {
        boolean fin;
        xError = Math.abs(x - arm.getArmXPosition());
        zError = Math.abs(z - arm.getArmZPosition());
        distError = Math.sqrt((xError * xError) + (zError * zError));
        SmartDashboard.putNumber("Arm Dist Error", distError);

        if(DriverStation.isAutonomous()) {
            fin = (distError < Constants.ARM_ACCEPT_ERROR);
        } else {
            fin = false;
        }
        return fin; 
    }

    @Override
    public void end(boolean interrupted) {
        arm.setShoulderMotorVolts(0);
        arm.setElbowMotorVolts(0);
    }
}