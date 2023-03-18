package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.interfaces.IOperatorControls;

public class RumbleSequence extends CommandBase {
    private double count;
    private IOperatorControls cont;

    public RumbleSequence(IOperatorControls cont){
        this.cont = cont;
        this.count = 0;
    }

    @Override
    public void initialize() {
        count = 0;
    }

    @Override
    public void execute() {
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


    @Override
    public boolean isFinished() {
        if(count >= 50) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        cont.setBothRumble(0);
    }
}
