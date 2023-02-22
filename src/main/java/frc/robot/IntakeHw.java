package frc.robot;

import frc.robot.interfaces.IIntakeControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class IntakeHw implements IIntakeControl{
    TalonSRX pivotMotor;
    DutyCycle pivotEncoder;
    double currentIntakeDeg;
    PIDController pid;
    
    public IntakeHw(){
        pivotMotor = new TalonSRX(47);
        pivotMotor.setNeutralMode(NeutralMode.Brake);
        pivotEncoder = new DutyCycle(new DigitalInput(2)); //channel is currently 3 on PDP schematic but tail encoder is plugged in there

        pid = new PIDController(0.13, 0.005, 0);
    }
    
    public void setPivotAngle(double angleDeg){
        double volts = pid.calculate(currentIntakeDeg,angleDeg);
        pivotMotor.set(ControlMode.PercentOutput, volts / Robot.BatteryVoltage());
    }

    public void setPivotMotorVolts(double volts){
        pivotMotor.set(ControlMode.PercentOutput, volts/Robot.BatteryVoltage());
    }

    public double getPivotAngle() {
        return currentIntakeDeg;
    }

    public void updateInputs(){                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        //Frac is 0 at lowest point, 1 at max extension
        var rawDC = pivotEncoder.getOutput();
        currentIntakeDeg = MathUtil.inputModulus((rawDC * 360) - Constants.INTAKE_OFFSET,-180,180);
        pivotMotor.setSelectedSensorPosition(currentIntakeDeg);
    }
}
