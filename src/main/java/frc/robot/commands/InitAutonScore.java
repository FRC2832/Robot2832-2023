package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.Intake;
import frc.robot.commands.ArmAutonPoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class InitAutonScore extends CommandBase {
    private Arm arm;
    private Intake intake;
    private int location;

    public InitAutonScore(Arm arm, Intake intake, int location) {
        this.arm = arm;
        this.intake = intake;
        this.location = location;
        addRequirements(arm, intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
         
        new SequentialCommandGroup(
        //Score to top
        new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z),
        //Release cone
        new IntakeBackward(this.intake),
        //Reset arm position
        new ArmAutonPoint(this.arm, Constants.ArmToSecureLocation_X, Constants.ArmToSecureLocation_Z)
        );

    }

    @Override
    public boolean isFinished() {
            return false;
    }

    @Override
    public void end(boolean interrupted) {}
}