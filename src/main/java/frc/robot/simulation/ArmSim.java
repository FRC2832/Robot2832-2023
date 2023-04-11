package frc.robot.simulation;

import org.livoniawarriors.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.interfaces.IArmControl;

public class ArmSim implements IArmControl {
    private double elbowDeg;
    private double shoulderDeg;

    private double elbowVolts;
    private double shoulderVolts;

    private ProfiledPIDController elbowPid;
    private boolean elbowPidActive; 

    private ProfiledPIDController shoulderPid;
    private boolean shoulderPidActive; 

    private final double kV_Elbow = 0.0838;         //4v for 2.5 sec/119.3*
    private final double kV_Shoulder = 0.07832;     //4v for 1.6 sec/81.765*

    public ArmSim() {
        shoulderDeg = 45;
        elbowDeg = 120;

        elbowPidActive = false;
        elbowPid = new ProfiledPIDController(0.28, 0, 0, new Constraints(200, 300));

        shoulderPidActive = false;
        shoulderPid = new ProfiledPIDController(0.12, 0, 0, new Constraints(80, 70));

        Logger.RegisterSensor("Shoulder Angle", () -> getShoulderAngle());
        Logger.RegisterSensor("Elbow Angle", () -> getElbowAngle());
    }

    public void updateInputs() {
        if(elbowPidActive) {
            elbowVolts = -elbowPid.calculate(elbowDeg);
            elbowPidActive = false;
        } else {
            elbowPid.reset(elbowDeg);
        }
        elbowDeg -= elbowVolts * Constants.LOOP_TIME / kV_Elbow;
        elbowVolts = 0;

        if(shoulderPidActive) {
            shoulderVolts = shoulderPid.calculate(shoulderDeg);
            shoulderPidActive = false;
        } else {
            shoulderPid.reset(shoulderDeg);
        }
        shoulderDeg += shoulderVolts * Constants.LOOP_TIME / kV_Shoulder;
        shoulderVolts = 0;
    }

    @Override
    public void setShoulderMotorVolts(double volts) {
        shoulderVolts = volts;
    }

    @Override
    public void setElbowMotorVolts(double volts) {
        elbowVolts = volts;
    }

    @Override
    public double getElbowAngle() {
        return MathUtil.inputModulus(elbowDeg, -180, 180);
    }

    @Override
    public double getShoulderAngle() {
        return MathUtil.inputModulus(shoulderDeg, -90, 270);
    }

    @Override
    public void checkBrake() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getFeedForward(double offset) {
        return 0;
    }

    @Override
    public void setElbowAngle(double angleDeg) {
        elbowPidActive = true;
        elbowPid.setGoal(angleDeg);
    }

    @Override
    public void setShoulderAngle(double angleDeg) {
        shoulderPidActive = true;
        shoulderPid.setGoal(angleDeg);
    }

    @Override
    public double getElbowAbsAngle() {
        return elbowDeg;
    }

    @Override
    public double getShoulderAbsAngle() {
        return shoulderDeg;
    }
}
