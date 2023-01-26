package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Arm;


public class ArmToPoint extends CommandBase{
    private Arm arm;
    
    public ArmToPoint(Arm arm, double x, double z) {
        this.arm = arm;
        addRequirements(arm);
        SmartDashboard.putNumber("Arm Pos X", 36);
        SmartDashboard.putNumber("Arm Pos Z", 36);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        var x = SmartDashboard.getNumber("Arm Pos X", 36);
        var z = SmartDashboard.getNumber("Arm Pos Z", 36);

        double shoulder = 0;
        double elbow = 0;
        if(x >= 0) {
            double l = x;
            double h = Math.sqrt(l * l + z * z);
            double phi = Math.toDegrees(Math.atan(z/l));
            double theta = Math.toDegrees(Math.acos((h/2)/30));
            shoulder = phi + theta;
            elbow = (phi - theta);
        }
        else{
            double l = -x;
            double h = Math.sqrt(l * l + z * z);
            double phi = Math.toDegrees(Math.atan(z/l));
            double theta = Math.toDegrees(Math.acos((h/2)/30));
            shoulder = 180 - (phi + theta);
            elbow = 180 - (phi - theta);
        }
        arm.setElbowAngle(elbow);
        arm.setShoulderAngle(shoulder);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {}
}
