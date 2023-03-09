package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.ITailControl;


public class Tail extends SubsystemBase {
    private ITailControl hardware;
    double tailAngle;
    ProfiledPIDController tailPid;
   
    public Tail(ITailControl hardware) {
        super();
        this.hardware = hardware;
        tailPid = new ProfiledPIDController(0.23, 0.1, 0, 
            new Constraints(420, 420));
        hardware.updateInputs();
        tailPid.reset(hardware.getTailAngle());
        tailPid.setGoal(hardware.getTailAngle());
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
        if(!tailPid.atSetpoint()) {
            if(volts < 0) {
                volts -= 3; //compensate for the surgical tubing
            }
        } else {
            volts = -1;
        }
        setTailVoltage(volts);
    }

    public double getDistSensor() {
        return hardware.getDistSensor();
    }

}
