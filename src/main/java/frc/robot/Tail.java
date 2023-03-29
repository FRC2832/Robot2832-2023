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
    boolean isVoltControl;
    double requestedAngle;
   
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
        setTailAngle(Constants.TAIL_STOW_POINT);
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
        tailAngle = hardware.getTailAngle();
        if(!isVoltControl) {
            double volts = tailPid.calculate(tailAngle, requestedAngle);
            // if(tailPid.atSetpoint()) {
            //     volts = 0;
            // }
            setTailVoltage(volts);
        }
        isVoltControl = false;
    }

    public void setTailVoltage(double volts) {
        hardware.setTailVoltage(volts);
        isVoltControl = true;
    }

    public double getTailAngle() {
        return tailAngle;
    }

    public void setTailAngle(double angleDeg) {
        requestedAngle = angleDeg;
    }

    public double getDistSensor() {
        return hardware.getDistSensor();
    }

}
