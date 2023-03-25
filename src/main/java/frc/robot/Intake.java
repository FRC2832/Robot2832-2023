package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import org.livoniawarriors.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase { 
    private CANSparkMax intakeMotor;
    private double velocity;
    private Timer timer;

    public Intake() {
        super();
        intakeMotor = new CANSparkMax(8,MotorType.kBrushless);
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(15);
        timer = new Timer();

        Logger.RegisterCanSparkMax("Intake", intakeMotor);
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake Input Volts", intakeMotor.getBusVoltage());
    }

    public void setIntakeVolts(double volts) {
        intakeMotor.setVoltage(volts);
    }
    
    public void Grab(boolean forward) {
        velocity = intakeMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Intake Velocity", velocity);
        
        timer.start();
        if(timer.hasElapsed(1.0)) {
            intakeOff();
            timer.stop();
        } else {
            if (forward) {
                setIntakeVolts(Constants.IntakeVoltage);
            } else {
                setIntakeVolts(-1 * Constants.IntakeVoltage);
            }
        }
    }

    public void intakeOff(){
        setIntakeVolts(0.0);
    }

    public void resetTimer() {
        timer.reset();
    }
}
