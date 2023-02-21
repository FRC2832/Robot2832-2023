package frc.robot;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GrabberIntake implements Subsystem { 
    private CANSparkMax intakeMotor;
    private double velocity;
    private Timer timer;
    private boolean done;

    public GrabberIntake() {
        intakeMotor = new CANSparkMax(48,MotorType.kBrushless);
        intakeMotor.setInverted(true);
        timer = new Timer();
        done = false;
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake Speed%", intakeMotor.get());
    }

    public void setIntakeVolts(double volts) {
        intakeMotor.setVoltage(volts);
        SmartDashboard.putNumber("Intake Volts", volts);
    }
    
    public void Grab(boolean forward) {
        // if(forward && done) {
        //     intakeOff();
        //     return;
        // }
        // else {
        //     done = false;
        // }

        velocity = intakeMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Intake Velocity", velocity);
        
        //if(Math.abs(velocity) > 0) {
        timer.start();
        if(timer.hasElapsed(1.0)) {
            intakeOff();
            //done = true;
            timer.stop();
            //timer.reset();
        } else {
            if (forward) {
                setIntakeVolts(Constants.IntakeVoltage);
            } else {
                setIntakeVolts(-1 * Constants.IntakeVoltage);
            }
        }
        // }
        // else {
        //     timer.stop();
        //     timer.reset();
        // }
    }

    public void intakeOff(){
        setIntakeVolts(0.0);
    }

    public void resetTimer() {
        timer.reset();
    }
}
