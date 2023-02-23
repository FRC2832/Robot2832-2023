package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Tail;
import frc.robot.interfaces.IDriveControls;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TailMovement extends CommandBase{
    private Tail tail;
    private IDriveControls controls;


    public TailMovement(IDriveControls controls, Tail tail){
        this.tail = tail;
        this.controls = controls;
        addRequirements(tail);
    }


    @Override
    public void initialize() {}


    @Override
    public void execute() {  
        if(controls.TailUpRequested().getAsBoolean()){
            tail.setTailAngle(Constants.TAIL_HIGH_POINT);
        } else if(controls.TailDownRequested().getAsBoolean()){
            tail.setTailAngle(Constants.TAIL_LOW_POINT);
        }  else {
            tail.setTailVoltage(0);
        }
    }


    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void end(boolean interrupted) {
    }
}
