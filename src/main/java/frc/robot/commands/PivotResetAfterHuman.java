package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.Robot;

public class PivotResetAfterHuman extends CommandBase {
    double scoreX;
    double scoreZ;
    double resetAngle;

    public PivotResetAfterHuman(Arm arm, double scoreX, double scoreZ){
        this.scoreX = scoreX;
        this.scoreZ = scoreZ;
        resetAngle = Constants.PivotToPickupHuman;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() { 
        boolean pieceMode = Robot.getGamePieceMode();

        if(scoreX == Constants.ArmToScoreTop_X && scoreZ == Constants.ArmToScoreTop_Z) {
            resetAngle = pieceMode ? Constants.CubePivotResetAngleScoreTop : Constants.ConePivotResetAngleScoreTop;
        } else if(scoreX == Constants.ArmToScoreMiddleFront_X && scoreZ == Constants.ArmToScoreMiddleFront_Z) {
            resetAngle = pieceMode ? Constants.CubePivotResetAngleScoreMiddleFront : Constants.ConePivotResetAngleScoreMiddleFront;
        } else if(scoreX == Constants.ArmToScoreMiddle_X && scoreZ == Constants.ArmToScoreMiddle_Z) {
            resetAngle = pieceMode ? Constants.CubePivotResetAngleScoreMiddle : Constants.ConePivotResetAngleScoreMiddle;
        } else {
            resetAngle = pieceMode ? Constants.CubePivotResetAngleScoreLow : Constants.ConePivotResetAngleScoreLow;
        }
        Robot.pivot.setPivotAngle(resetAngle);
    }


    @Override
    public boolean isFinished() {
        return Math.abs(Robot.pivot.getPivotAngle() - resetAngle) < 5;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
