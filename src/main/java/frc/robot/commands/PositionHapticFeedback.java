package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.controls.LilJimmyDriveControls;
import edu.wpi.first.wpilibj.Timer;

public class PositionHapticFeedback extends CommandBase {
    private double xPos;
    private double zPos;
    private XboxController cont;
    private Arm arm;
    private Timer timer;

    public PositionHapticFeedback(double xPos, double zPos, Arm arm){
        this.xPos = xPos;
        this.zPos = zPos;
        this.arm = arm;
        this.timer = new Timer();
        cont = LilJimmyDriveControls.operCont;
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        double xError = Math.abs(xPos - arm.getArmXPosition());
        double zError = Math.abs(zPos - arm.getArmZPosition());
        double distError = Math.sqrt((xError * xError) + (zError * zError));
        boolean fin = (distError < Constants.ARM_ACCEPT_ERROR);
        if(fin){
            timer.start();
            // vibrate operator controller
            cont.setRumble(RumbleType.kLeftRumble, 1.0);
            cont.setRumble(RumbleType.kRightRumble, 1.0);
        }
    }

    @Override
    public boolean isFinished() {
        if(timer.hasElapsed(1.0)) {
            cont.setRumble(RumbleType.kLeftRumble, 0);
            cont.setRumble(RumbleType.kRightRumble, 0);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
