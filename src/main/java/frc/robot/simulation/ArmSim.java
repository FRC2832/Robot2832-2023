package frc.robot.simulation;

import org.livoniawarriors.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.interfaces.IArmControl;

public class ArmSim implements IArmControl {
    private double elbowDeg;
    private double shoulderDeg;

    private double elbowVolts;
    private double shoulderVolts;

    private final double kV_Elbow = 0.0838;         //4v for 2.5 sec/119.3*
    private final double kV_Shoulder = 0.07832;     //4v for 1.6 sec/81.765*

    public ArmSim() {
        shoulderDeg = 45;
        elbowDeg = 120;

        Logger.RegisterSensor("Shoulder Angle", () -> getShoulderAngle());
        Logger.RegisterSensor("Elbow Angle", () -> getElbowAngle());
    }

    public void updateInputs() {
        elbowDeg -= elbowVolts * Constants.LOOP_TIME / kV_Elbow;
        elbowVolts = 0;

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
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setShoulderAngle(double angleDeg) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getElbowAbsAngle() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getShoulderAbsAngle() {
        // TODO Auto-generated method stub
        return 0;
    }
}
