package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.ITailControl;


public class Tail extends SubsystemBase {
    private ITailControl hardware;
    double tailAngle;
   
    public Tail(ITailControl hardware) {
        super();
        this.hardware = hardware;
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
    }

    public void setTailVoltage(double volts) {
        hardware.setTailVoltage(volts);
    }

    public double getTailAngle() {
        tailAngle = hardware.getTailAngle();
        return tailAngle;
    }

    public void setTailAngle(double angleDeg) {
        hardware.setTailAngle(angleDeg);
    }

    public double getDistSensor() {
        return hardware.getDistSensor();
    }

}
