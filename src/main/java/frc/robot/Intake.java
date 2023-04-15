package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.IOperatorControls;
import edu.wpi.first.wpilibj.DriverStation;
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
    private IOperatorControls controls;
    private boolean hasPiece;
    private double motorTime;

    public Intake(IOperatorControls controls) {
        super();
        intakeMotor = new CANSparkMax(8,MotorType.kBrushless);
        intakeMotor.setInverted(true);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(30);
        timer = new Timer();
        this.controls = controls;
        hasPiece = true;

        Logger.RegisterCanSparkMax("Intake", intakeMotor);
    }

    public void setOperator(IOperatorControls controls) {
        this.controls = controls;
    }

    public void periodic() {
        SmartDashboard.putNumber("Intake Input Volts", intakeMotor.getBusVoltage());
        SmartDashboard.putBoolean("Piece Detected", hasPiece);

        if(!DriverStation.isDisabled()){
            if(controls.IntakeSuckRequested().getAsBoolean()){
                if(motorTime>10){
                    hasPiece=true;
                } else if (getIntakeCurrent()>=20) {
                    motorTime++;
                } else {
                }
            }
            else if(controls.IntakeSpitRequested().getAsBoolean()){
                hasPiece = false;
            } else {
                motorTime = 0;
            }
        }
    }

    public void setIntakeVolts(double volts) {
        intakeMotor.setVoltage(volts);
    }

    public double getIntakeCurrent() {
        return intakeMotor.getOutputCurrent();  
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

    public boolean HasPiece() {
        return hasPiece;
    }

    public void intakeOff(){
        setIntakeVolts(0.0);
    }

    public void resetTimer() {
        timer.reset();
    }
}
