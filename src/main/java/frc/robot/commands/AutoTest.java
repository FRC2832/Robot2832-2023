package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Arm;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.interfaces.ISwerveDriveIo;

public class AutoTest extends CommandBase{
    Command tests;
    ISwerveDrive drive;
    Arm arm;
    static String results;

    public AutoTest(ISwerveDrive drive, Arm arm) {
        this.drive = drive;
        this.arm = arm;
        results = "";
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    public static void AddMessage(String msg) {
        if (results == null) {
            results = "msg";
        }
        results += msg + "\n";
    }

    public static String GetMessage() {
        return results;
    }

    @Override
    public void initialize() {
        tests = new SequentialCommandGroup(
            new FlashDriveTest(),
            (new SwerveDriveTest(drive,0)).withTimeout(3),
            (new SwerveDriveTest(drive,1)).withTimeout(3),
            (new SwerveDriveTest(drive,2)).withTimeout(3),
            (new SwerveDriveTest(drive,3)).withTimeout(3),
            (new SwerveTurnTest(drive,0)).withTimeout(3),
            (new SwerveTurnTest(drive,1)).withTimeout(3),
            (new SwerveTurnTest(drive,2)).withTimeout(3),
            (new SwerveTurnTest(drive,3)).withTimeout(3),
            (new ArmShoulderTest(arm)).withTimeout(3),
            (new ArmElbowTest(arm)).withTimeout(3)
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
    public void end(boolean interrupted) {
        System.out.print(GetMessage());
    }

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
                AutoTest.AddMessage("Swerve drive wheel " + wheel + " test failed.");
            } else {
                AutoTest.AddMessage("Swerve drive wheel " + wheel + " test passed.");
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
                AutoTest.AddMessage("Swerve turn wheel " + wheel + " test failed.");
            } else {
                AutoTest.AddMessage("Swerve turn wheel " + wheel + " test passed.");
            }
        }
    }

    private class FlashDriveTest extends CommandBase {
        @Override
        public boolean isFinished() {
            boolean retVal = false;
            try {
                Path usbDir;
                usbDir = Paths.get("/u").toRealPath();
                retVal = Files.isWritable(usbDir);
            } catch (IOException e) {
            }

            if (retVal) {
                AutoTest.AddMessage("Flash drive plugged in.");
            } else {
                AutoTest.AddMessage("Flash drive MISSING.");
            }
            return true;
        }
    }

    private class ArmShoulderTest extends CommandBase {
        Arm arm;
        double startAngle;

        public ArmShoulderTest(Arm arm) {
            this.arm = arm;
            addRequirements(arm);
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public void initialize() {
            startAngle = arm.getShoulderAngle();
        }

        @Override
        public void execute() {
            arm.periodic();
            arm.setShoulderMotorVolts(1);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(arm.getShoulderAngle() - startAngle) > 3;
        }

        @Override
        public void end(boolean interrupted) {
            arm.setShoulderMotorVolts(0);
            if(interrupted) {
                AutoTest.AddMessage("Shoulder test failed.");
            } else {
                AutoTest.AddMessage("Shoulder test passed.");
            }
        }
    } 

    private class ArmElbowTest extends CommandBase {
        Arm arm;
        double startAngle;

        public ArmElbowTest(Arm arm) {
            this.arm = arm;
            addRequirements(arm);
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public void initialize() {
            startAngle = arm.getElbowAngle();
        }

        @Override
        public void execute() {
            arm.periodic();
            arm.setElbowMotorVolts(1);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(arm.getElbowAngle() - startAngle) > 3;
        }

        @Override
        public void end(boolean interrupted) {
            arm.setElbowMotorVolts(0);
            if(interrupted) {
                AutoTest.AddMessage("Elbow test failed.");
            } else {
                AutoTest.AddMessage("Elbow test passed.");
            }
        }
    } 

    //tests still needed
    //compressor
    //tail (abs)
    //pivot (abs)
    //intake (check velocity)
    //rio voltages
    //faults

    //end of class
}
