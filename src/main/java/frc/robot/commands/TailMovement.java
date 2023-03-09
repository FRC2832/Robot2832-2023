package frc.robot.commands;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.Tail;
import frc.robot.interfaces.IDriveControls;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TailMovement extends CommandBase{
    private Tail tail;
    private Arm arm;
    private IDriveControls controls;

    public TailMovement(IDriveControls controls, Tail tail, Arm arm){
        this.tail = tail;
        this.controls = controls;
        this.arm = arm;
        addRequirements(tail);
    }

    @Override
    public void initialize() {}


    @Override
    public void execute() {  
        //auto control
        boolean tailUpOverride = SmartDashboard.getBoolean("Distance Sensor Not Working (Override Tail Up)", false);
        double tailDist = tail.getDistSensor();

        if(arm.getArmXPosition() <= -10) {
            tail.setTailAngle(Constants.TAIL_HIGH_POINT);
        } else if(controls.TailUpRequested().getAsBoolean()){
            //tail.setTailVoltage(3);
            tail.setTailAngle(Constants.TAIL_HIGH_POINT);
        } else if(controls.TailDownRequested().getAsBoolean()){
            tail.setTailAngle(Constants.TAIL_LOW_POINT);
            //tail.setTailVoltage(-5);
        }  else if(tailDist > 0 && tailDist < 5.3 && !tailUpOverride) {
            tail.setTailAngle(Constants.TAIL_HIGH_POINT);
        } else {
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
