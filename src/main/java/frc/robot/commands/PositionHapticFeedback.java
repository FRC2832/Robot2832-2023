package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.interfaces.IOperatorControls;
import edu.wpi.first.wpilibj.Timer;

public class PositionHapticFeedback extends CommandBase {
    private double xPos;
    private double zPos;
    private IOperatorControls cont;
    private Arm arm;
    private Timer timer;
    private double count;

    public PositionHapticFeedback(double xPos, double zPos, Arm arm, IOperatorControls cont){
        this.xPos = xPos;
        this.zPos = zPos;
        this.arm = arm;
        this.timer = new Timer();
        this.cont = cont;
        this.count = 0;
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
            RumbleSequence();
        }
    }

    @Override
    public boolean isFinished() {
        if(timer.hasElapsed(1.0)) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        cont.setBothRumble(0.0);
    }

    private void RumbleSequence(){
        count++;
        if(count < 5){
            cont.setLeftRumble(0.5);
        }
        else if(count < 10){
            cont.setLeftRumble(0);
            cont.setRightRumble(0.5);
        }
        else if(count < 15){
            cont.setRightRumble(0);
            cont.setLeftRumble(0.5);
        }
        else if(count < 20){
            cont.setLeftRumble(0);
            cont.setRightRumble(0.5);
        }
        else if(count < 25){
            cont.setRightRumble(0);
            cont.setLeftRumble(0.5);
        }
        else if(count < 30){
            cont.setLeftRumble(0);
            cont.setRightRumble(0.5);
        }
        else if(count < 35){
            cont.setRightRumble(0);
            cont.setLeftRumble(0.5);
        }
        else if(count < 40){
            cont.setLeftRumble(0);
            cont.setRightRumble(0.5);
        }
        else if(count < 45){
            cont.setRightRumble(0);
            cont.setLeftRumble(0.5);
        }
        else if(count < 50){
            cont.setLeftRumble(0);
            cont.setRightRumble(0.5);
        }
        else{
            cont.setBothRumble(0);
        }
    }
}
