package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.commands.ArmAutonPoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class InitAutonScore extends CommandBase {
    private Arm arm;
    private int location;

    public InitAutonScore(Arm arm, int location) {
        this.arm = arm;
        this.location = location;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(this.location == 0) { //Score to top
            new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z)
        } else if(this.location == 1) { //Score to middle
            new ArmAutonPoint(this.arm, Constants.ArmToScoreMiddle_X, Constants.ArmToScoreMiddle_Z)
        } else { //Score to low
            new ArmAutonPoint(this.arm, Constants.ArmToScoreLow_X, Constants.ArmToScoreLow_Z)
        }

        //TODO: Release cone

        //Reset arm position
        new ArmAutonPoint(this.arm, Constants.ArmToSecureLocation_X, Constants.ArmToSecureLocation_Z);
    }

    @Override
    public boolean isFinished() {

    }

    @Override
    public void end(boolean interrupted) {}
}