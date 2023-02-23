package frc.robot.simulation;

import frc.robot.interfaces.ITailControl;

public class TailSim implements ITailControl{
    double tailAngle;
    double distValue;

    final double TAIL_ZERO_OFFSET = -340+12;
    final double COUNTS_PER_DEGREE_TAIL = 101;

    public TailSim(){
    }

    @Override
    public void setTailVoltage(double volts) {
        
    }

    @Override
    public void setTailAngle(double angleDeg) {
    }

    @Override
    public double getTailAngle() {
        return tailAngle;
    }

    

    @Override
    public void updateInputs() {
        
    }

    @Override
    public double getDistSensor() {
        return distValue;
    }
}
