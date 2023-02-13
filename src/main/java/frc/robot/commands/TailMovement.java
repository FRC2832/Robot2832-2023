package frc.robot.commands;
import frc.robot.Tail;
import frc.robot.TailHw;
import frc.robot.interfaces.IDriveControls;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TailMovement extends CommandBase{
    private Tail tail;
    private IDriveControls controls;


    public TailMovement(IDriveControls controls){
        this.tail = new Tail(new TailHw());
        this.controls = controls;
        addRequirements(tail);
    }


    @Override
    public void initialize() {}


    @Override
    public void execute() {  
        if(controls.TailUpRequested().getAsBoolean()){
            tail.setTailAngle(180); // TODO: Determine angle of "UP" position
        }  
        if(controls.TailDownRequested().getAsBoolean()){
            tail.setTailAngle(0); // TODO: Determine angle of "DOWN" position
        }  
    }


    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void end(boolean interrupted) {}
}
