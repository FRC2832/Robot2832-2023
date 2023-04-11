package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.ITailControl;


public class Tail extends SubsystemBase {
    private ITailControl hardware;
    double tailAngle;
    ProfiledPIDController tailPid;
    boolean isVoltControl;
    double requestedAngle;
   
    public Tail(ITailControl hardware) {
        super();
        this.hardware = hardware;
        tailPid = new ProfiledPIDController(0.24, 0.028, 0, 
            new Constraints(200, 300));
        //tailPid = new PIDController(0.06, 0.028, 0);
        hardware.updateInputs();
        //tailPid.reset();
        tailPid.reset(hardware.getTailAngle());
        tailPid.setGoal(hardware.getTailAngle());
        tailPid.setTolerance(3);
        requestedAngle = Constants.TAIL_STOW_POINT;
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
        tailAngle = hardware.getTailAngle();
        if(!isVoltControl) {
            double ff;
            if(HasPiece()) {
                //20% power to hold up piece
                ff = 12.16 * 0.2 * Math.cos(Math.toRadians(hardware.getTailAngle()));
            } else {
                //10% power for no piece
                ff = 12.16 * 0.1 * Math.cos(Math.toRadians(hardware.getTailAngle()));
            }
            double volts = tailPid.calculate(tailAngle, requestedAngle) + ff;
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

    public boolean HasPiece() {
        var tailDist = hardware.getDistSensor();
        return tailDist > 0 && tailDist < 5.3;
    }
}
