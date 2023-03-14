package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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

    public PickUpHapticFeedback(IOperatorControls operatorCont, IDriveControls driverCont, Intake intake){
        this.timer = new Timer();
        this.operatorCont = operatorCont;
        this.driverCont = driverCont;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if(intake.getCurrent() > Constants.INTAKE_CURRENT_THRESHOLD){
            timer.start();
            // vibrate operator controller (right side)
            operatorCont.getCont().setRumble(RumbleType.kRightRumble, 1.0);

            // vibrate driver controller (right side)
            driverCont.getCont().setRumble(RumbleType.kRightRumble, 1.0);
        }
    }

    @Override
    public boolean isFinished() {
        if(timer.hasElapsed(1.0)) {
            operatorCont.getCont().setRumble(RumbleType.kRightRumble, 0);
            driverCont.getCont().setRumble(RumbleType.kRightRumble, 0);
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}