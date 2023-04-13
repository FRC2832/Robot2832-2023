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
    
    public ArmAutonPoint(Arm arm, double x, double z) {
        this.arm = arm;
        this.x = x;
        zOrig = z;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        if(Robot.getGamePieceMode() == Robot.CUBE_MODE){ // if cube mode true: lower z by 4
            z = zOrig - Constants.ScoreCubeOffset;
        } else {
            z = zOrig;
        }
        arm.resetPids();
    }

    @Override
    public void execute() {
        arm.calcAngles(x, z);
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