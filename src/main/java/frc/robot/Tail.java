package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.interfaces.ITailControl;


public class Tail implements Subsystem{
    private ITailControl hardware;
    double tailAngle;
   
    public Tail(ITailControl hardware) {
        this.hardware = hardware;
    }


    @Override
    public void periodic() {
        hardware.updateInputs();
        SmartDashboard.putNumber("Tail Angle", getTailAngle());
    }


    private double getTailAngle() {
        tailAngle = hardware.getTailAngle();
        return tailAngle;
    }


    public void setTailAngle(double angleDeg) {
        hardware.setTailAngle(angleDeg);
    }
}
