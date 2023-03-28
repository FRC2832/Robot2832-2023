package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.ITailControl;


public class Tail extends SubsystemBase {
    private ITailControl hardware;
    double tailAngle;
    PIDController tailPid;
   
    public Tail(ITailControl hardware) {
        super();
        this.hardware = hardware;
        //tailPid = new ProfiledPIDController(0.03, 0.007, 0, 
        //    new Constraints(120, 60));
        tailPid = new PIDController(0.04, 0.007, 0);
        hardware.updateInputs();
        tailPid.reset();
        //tailPid.reset(hardware.getTailAngle());
        //tailPid.setGoal(hardware.getTailAngle());
        tailPid.setTolerance(3);
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
        if(tailPid.atSetpoint()) {
            volts = 0;
        }
        setTailVoltage(volts);
    }

    public double getDistSensor() {
        return hardware.getDistSensor();
    }

}
