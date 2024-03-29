package frc.robot;

import org.livoniawarriors.Logger;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
//import edu.wpi.first.wpilibj.RobotController;
import frc.robot.interfaces.ITailControl;


public class TailHw implements ITailControl{
    //TalonSRX tailMotor;
    DutyCycle tailEncoder;
    Rev2mDistanceSensor distSensor;

    double tailAngle;
    double distValue;

    final double TAIL_ZERO_OFFSET = -340+146-22;
    final double COUNTS_PER_DEGREE_TAIL = 101;

    public TailHw(){
        //tailMotor = new TalonSRX(49);
        //tailMotor.setNeutralMode(NeutralMode.Brake);
        //tailMotor.setInverted(false);
        tailEncoder = new DutyCycle(new DigitalInput(6));
        
        distSensor = new Rev2mDistanceSensor(Port.kOnboard);
        distSensor.setAutomaticMode(true);
        distSensor.setMeasurementPeriod(0.018);

        //Logger.RegisterTalon("Tail", tailMotor);
        Logger.RegisterSensor("Tail Angle", () -> getTailAngle());
        Logger.RegisterSensor("Tail Dist", () -> getDistSensor());
    }

    @Override
    public void setTailVoltage(double volts) {
        if(!DriverStation.isTest()) {
            //tailMotor.set(ControlMode.PercentOutput, -volts/RobotController.getBatteryVoltage());
        } else {
            //tailMotor.set(ControlMode.PercentOutput, 0);
        }
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
    }

    @Override
    public double getDistSensor() {
        return distValue;
    }
}
