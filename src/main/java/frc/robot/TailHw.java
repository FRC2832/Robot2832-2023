package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.interfaces.ITailControl;


public class TailHw implements ITailControl{
    TalonSRX tailMotor;
    DutyCycle tailEncoder;
    Rev2mDistanceSensor distSensor;
    PIDController tailPid;

    double tailAngle;
    double distValue;

    final double TAIL_ZERO_OFFSET = -340+12;
    final double COUNTS_PER_DEGREE_TAIL = 101;

    public TailHw(){
        tailMotor = new TalonSRX(49);
        tailMotor.setNeutralMode(NeutralMode.Brake);
        tailEncoder = new DutyCycle(new DigitalInput(3)); // TODO: Verify channel number
        
        distSensor = new Rev2mDistanceSensor(Port.kOnboard);
        distSensor.setAutomaticMode(true);
        distSensor.setMeasurementPeriod(0.018);

        tailPid = new PIDController(0.2, 0, 0);
    }

    @Override
    public void setTailVoltage(double volts) {
        tailMotor.set(ControlMode.PercentOutput, volts/Constants.NOM_BATTERY_VOLTAGE);
    }

    @Override
    public void setTailAngle(double angleDeg) {
        double volts = tailPid.calculate(tailAngle, angleDeg);
        setTailVoltage(volts);
    }

    @Override
    public double getTailAngle() {
        return tailAngle;
    }

    @Override
    public void updateInputs() {
        tailAngle = TAIL_ZERO_OFFSET + tailEncoder.getOutput() * 360;
        tailAngle = MathUtil.inputModulus(tailAngle, -180, 180);
        distValue = distSensor.getRange(Unit.kInches);
        tailMotor.setSelectedSensorPosition(tailAngle * COUNTS_PER_DEGREE_TAIL);
    }

    @Override
    public double getDistSensor() {
        return distValue;
    }
}
