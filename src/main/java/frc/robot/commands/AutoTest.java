package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.interfaces.ISwerveDriveIo;

public class AutoTest extends CommandBase{
    Command tests;
    ISwerveDrive drive;

    public AutoTest(ISwerveDrive drive) {
        this.drive = drive;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        tests = new SequentialCommandGroup(
            (new SwerveDriveTest(drive,0)).withTimeout(3),
            (new SwerveDriveTest(drive,1)).withTimeout(3),
            (new SwerveDriveTest(drive,2)).withTimeout(3),
            (new SwerveDriveTest(drive,3)).withTimeout(3),
            (new SwerveTurnTest(drive,0)).withTimeout(3),
            (new SwerveTurnTest(drive,1)).withTimeout(3),
            (new SwerveTurnTest(drive,2)).withTimeout(3),
            (new SwerveTurnTest(drive,3)).withTimeout(3)
        );

        //CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().schedule(tests);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return tests.isFinished();
    }

    @Override
    public void end(boolean interrupted) {}

    private class SwerveDriveTest extends CommandBase {
        ISwerveDriveIo hardware;
        int wheel;
        double startDist;

        public SwerveDriveTest(ISwerveDrive drive, int wheel) {
            hardware = drive.getHardware();
            this.wheel = wheel;
            addRequirements(drive);
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public void initialize() {
            startDist = hardware.getCornerDistance(wheel);
        }

        @Override
        public void execute() {
            hardware.updateInputs();
            hardware.setDriveCommand(wheel, ControlMode.PercentOutput, 1./RobotController.getBatteryVoltage());
        }

        @Override
        public boolean isFinished() {
            return hardware.getCornerDistance(wheel)-startDist > 0.05;
        }

        @Override
        public void end(boolean interrupted) {
            hardware.setDriveCommand(wheel, ControlMode.Disabled, 0);
            if(interrupted) {
                System.out.println("Swerve drive wheel " + wheel + " test failed.");
            } else {
                System.out.println("Swerve drive wheel " + wheel + " test passed.");
            }
        }
    } 

    private class SwerveTurnTest extends CommandBase {
        ISwerveDriveIo hardware;
        int wheel;
        double startAbsAngle;
        double startMotorAngle;

        public SwerveTurnTest(ISwerveDrive drive, int wheel) {
            hardware = drive.getHardware();
            this.wheel = wheel;
            addRequirements(drive);
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public void initialize() {
            startAbsAngle = hardware.getCornerAbsAngle(wheel);
            startMotorAngle = hardware.getCornerAngle(wheel);
        }

        @Override
        public void execute() {
            hardware.updateInputs();
            hardware.setTurnCommand(wheel, ControlMode.PercentOutput, -1./RobotController.getBatteryVoltage());
        }

        @Override
        public boolean isFinished() {
            return hardware.getCornerAbsAngle(wheel)-startAbsAngle > 3 
                && hardware.getCornerAngle(wheel)-startMotorAngle > 3;
        }

        @Override
        public void end(boolean interrupted) {
            hardware.setTurnCommand(wheel, ControlMode.Disabled, 0);
            if(interrupted) {
                System.out.println("Swerve turn wheel " + wheel + " test failed.");
            } else {
                System.out.println("Swerve turn wheel " + wheel + " test passed.");
            }
        }
    } 
}
