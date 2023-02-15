package frc.robot;

import frc.robot.interfaces.IIntakeControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class IntakeHw implements IIntakeControl{
    TalonSRX pivotMotor;
    DutyCycle pivotEncoder;
    double currentIntakeDeg;

    final double MIN_DUTY_CYCLE = 0.998;
    final double MAX_DUTY_CYCLE = 0.167;
    final double INTAKE_START_ANGLE = 90; // the angle that the sensor returns when intake is at the starting position
    final double INTAKE_MIN_ANGLE = -90 + INTAKE_START_ANGLE; // TODO: Determine correct angle
    final double INTAKE_MAX_ANGLE = 90 + INTAKE_START_ANGLE; // TODO: Determine correct angle
    

    public IntakeHw(){
        pivotMotor = new TalonSRX(47);
        pivotMotor.setNeutralMode(NeutralMode.Brake);
        pivotEncoder = new DutyCycle(new DigitalInput(2)); //channel is currently 3 on PDP schematic but tail encoder is plugged in there
    }
    
    public void setPivotAngle(double angleDeg){
        if(INTAKE_MIN_ANGLE < angleDeg && INTAKE_MAX_ANGLE > angleDeg){
            pivotMotor.set(ControlMode.Position, angleDeg);
            currentIntakeDeg = angleDeg;
        }
    }

    public void setPivotMotorVolts(double volts){
        pivotMotor.set(ControlMode.PercentOutput, volts/Constants.NOM_BATTERY_VOLTAGE);
    }

    public double getPivotAngle(){
        return currentIntakeDeg;
    }

    public void updateInputs(){                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        //Frac is 0 at lowest point, 1 at max extension
        var rawDC = pivotEncoder.getOutput();
        var dcFrac = (rawDC - MIN_DUTY_CYCLE) / (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);
        currentIntakeDeg = INTAKE_MIN_ANGLE + dcFrac * (INTAKE_MAX_ANGLE - INTAKE_MIN_ANGLE);

        pivotMotor.setSelectedSensorPosition(currentIntakeDeg);
    }
}
