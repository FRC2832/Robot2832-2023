package frc.robot.commands;
import frc.robot.Tail;
import frc.robot.TailHw;
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
            if(tail.getTailAngle() < 100){ //to facilitate the on/off toggle Jayden wants, if its already there it should go to the other position
                tail.setTailAngle(105);
            }
            else if(tail.getTailAngle() > 0){
                tail.setTailAngle(-8); //might mess up other drivers
            }
            /*
            if (tail.getTailAngle() < 150) {
                tail.setTailVoltage(6);
            } else{
                tail.setTailVoltage(0);
            }*/
        } else if(controls.TailDownRequested().getAsBoolean()){
            tail.setTailAngle(-8);
            /*
            if (tail.getTailAngle() > -8) {
                tail.setTailVoltage(-6);
            } else{
                tail.setTailVoltage(0);
            }
            */
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
