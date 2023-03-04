package frc.robot.simulation;

import org.livoniawarriors.Logger;

import frc.robot.Constants;
import frc.robot.interfaces.ITailControl;

public class TailSim implements ITailControl{
    double tailAngle = 105;
    double distValue = -1;
    double tailVolts = 0;

    final double kV = -0.02333;

    public TailSim(){
        Logger.RegisterSensor("Tail Angle", () -> getTailAngle());
        Logger.RegisterSensor("Tail Dist", () -> getDistSensor());
    }

    @Override
    public void setTailVoltage(double volts) {
        tailVolts = volts;
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
        tailAngle -= tailVolts * Constants.LOOP_TIME / kV;
        tailVolts = 0;
    }

    @Override
    public double getDistSensor() {
        return distValue;
    }
}
