package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.interfaces.IDriveControls;
import frc.robot.interfaces.IOperatorControls;
import edu.wpi.first.wpilibj.Timer; 
import frc.robot.Intake;

public class PickUpHapticFeedback extends CommandBase {
    private IOperatorControls operatorCont;
    private IDriveControls driverCont;
    private Timer timer;
    private Intake intake;
    private int count;

    public PickUpHapticFeedback(IOperatorControls operatorCont, IDriveControls driverCont, Intake intake){
        this.timer = new Timer();
        this.operatorCont = operatorCont;
        this.driverCont = driverCont;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        timer.reset();
        count = 0;
    }

    @Override
    public void execute() {
        if(intake.getCurrent() > Constants.INTAKE_CURRENT_THRESHOLD){
            timer.start();
            // vibrate operator controller (right side)
            RumbleSequence(operatorCont);

            // vibrate driver controller (right side)
            RumbleSequence(driverCont);
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
        operatorCont.setBothRumble(0.0);
        driverCont.setBothRumble(0.0);
    }

    private void RumbleSequence(IDriveControls cont){
        count++;
        if(count < 15){
            cont.setLeftRumble(0.75);
        }
        else if(count < 25){
            cont.setLeftRumble(0.0);
        }
        else if(count < 40){
            cont.setLeftRumble(0.8);
        }
        else if(count < 60){
            cont.setLeftRumble(0.0);
            cont.setRightRumble(1.0);
        }
    }

    private void RumbleSequence(IOperatorControls cont){
        count++;
        if(count < 15){
            cont.setLeftRumble(0.75);
        }
        else if(count < 25){
            cont.setLeftRumble(0.0);
        }
        else if(count < 40){
            cont.setLeftRumble(0.8);
        }
        else if(count < 60){
            cont.setLeftRumble(0.0);
            cont.setRightRumble(1.0);
        }
    }
}