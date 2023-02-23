package frc.robot;

import frc.robot.interfaces.IIntakeControl;

import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeHw implements IIntakeControl{
    TalonSRX pivotMotor;
    DutyCycle pivotEncoder;
    double currentIntakeDeg;
    PIDController pid;
    double rotations;
    double oldReading;

    public IntakeHw(){
        pivotMotor = new TalonSRX(47);
        pivotMotor.setNeutralMode(NeutralMode.Coast);
        pivotEncoder = new DutyCycle(new DigitalInput(2)); //channel is currently 3 on PDP schematic but tail encoder is plugged in there

        pid = new PIDController(0.13, 0.005, 0);
        rotations = 0;
        oldReading = 0;

        Logger.RegisterTalon("Pivot", pivotMotor);
        Logger.RegisterSensor("Pivot Angle", () -> getPivotAngle());
    }
    
    public void setPivotAngle(double angleDeg){
        double volts = pid.calculate(currentIntakeDeg,angleDeg);
        pivotMotor.set(ControlMode.PercentOutput, volts / RobotController.getBatteryVoltage());
    }

    public void setPivotMotorVolts(double volts){
        pivotMotor.set(ControlMode.PercentOutput, volts / RobotController.getBatteryVoltage());
    }

    public double getPivotAngle() {
        return currentIntakeDeg;
    }

    public void updateInputs(){                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        //Frac is 0 at lowest point, 1 at max extension
        var rawDC = pivotEncoder.getOutput();
        var newAngle = MathUtil.inputModulus((rawDC * 360) - Constants.INTAKE_OFFSET,0,360);

        //check to see if we go over a rotation and compensate with the absolute sensor
        var delta = newAngle - oldReading;
        if(Math.abs(delta) > 300) {
            rotations -= Math.signum(delta);
        }
        currentIntakeDeg = newAngle + (rotations * 360);
        pivotMotor.setSelectedSensorPosition(currentIntakeDeg);

        //save the reading for next loop
        oldReading = newAngle;
    }
}
