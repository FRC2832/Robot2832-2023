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

    public GrabberIntake(){
        
        intakeMotor = new CANSparkMax(48,MotorType.kBrushless);
        intakeMotor.setInverted(true);
        timer = new Timer();
        done = false;
    }
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed%", intakeMotor.get());

    }

    public void Grab(boolean forward) {
        if(forward && done) {
            intakeOff();
            return;
        }
        else {
            done = false;
        }

        if (forward) {
            intakeMotor.setVoltage(Constants.IntakeVoltage);
        } else {
            intakeMotor.setVoltage(-1 * Constants.IntakeVoltage);
        }
        
        velocity = intakeMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Intake Velocity", velocity);
        
        if(velocity >= 0) {
            timer.start();
            if(timer.hasElapsed(2.0)) {
                intakeOff();
                done = true;
            }
        }
        else {
            timer.stop();
            timer.reset();
        }
    }

    public void intakeOff(){
        intakeMotor.setVoltage(0.0);
        done = false;
    }

}
