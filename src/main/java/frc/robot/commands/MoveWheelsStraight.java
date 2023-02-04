package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.interfaces.ISwerveDrive;

public class MoveWheelsStraight extends CommandBase {
    private ISwerveDrive drive;

    public MoveWheelsStraight(ISwerveDrive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        SwerveModuleState[] states = new SwerveModuleState[Constants.NUM_WHEELS];
        states[ISwerveDrive.FL] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        states[ISwerveDrive.FR] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        states[ISwerveDrive.RL] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        states[ISwerveDrive.RR] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        drive.setWheelCommand(states);
    }

    @Override
    public boolean isFinished() {
        SwerveModulePosition[] states = drive.getSwerveStates();
        boolean result = true;
        for(int i=0; i<states.length; i++) {
            if (Math.abs(states[i].angle.getDegrees()) > 3) {
                result = false;
            }
        }
        return result; 
    }

    @Override
    public void end(boolean interrupted) {}
}
