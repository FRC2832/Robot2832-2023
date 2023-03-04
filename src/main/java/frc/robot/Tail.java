package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.ITailControl;


public class Tail extends SubsystemBase {
    private ITailControl hardware;
    double tailAngle;
    PIDController tailPid;
   
    public Tail(ITailControl hardware) {
        super();
        this.hardware = hardware;
        tailPid = new PIDController(0.13, 0, 0);
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
        double volts = tailPid.calculate(tailAngle, angleDeg);
        setTailVoltage(volts);
    }

    public double getDistSensor() {
        return hardware.getDistSensor();
    }

}
