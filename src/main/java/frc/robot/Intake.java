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
    private boolean forward;
    private double spitCount;
    private double suckDeadband;
    private double suckCount;

    public Intake(IOperatorControls controls) {
        super();
        intakeMotor = new CANSparkMax(8,MotorType.kBrushless);
        intakeMotor.setInverted(false);
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
        if(DriverStation.isAutonomous() && Math.abs(velocity) > 100){
            if(forward){ //forward = true means spitting piece
                suckDeadband = 0;
                suckCount = 0;
                spitCount++;
                if(spitCount > 10) {
                    hasPiece = false;
                }
            }
            if(!forward){
                spitCount = 0;
                suckDeadband++;
                if(suckDeadband > 15) {
                    if(suckCount>10){
                        hasPiece=true;
                    } else if (getIntakeCurrent()>=20) {
                        suckCount++;
                    } else {
                    }
                }
            }
        }
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
            // if(controls.IntakeSuckRequested().getAsBoolean()){
            //     if(motorTime>10){
            //         hasPiece=true;
            //     } else if (getIntakeCurrent()>=10) {
            //         motorTime++;
            //     } else {
            //     }
            // }
            // else if(controls.IntakeSpitRequested().getAsBoolean()){
            //     hasPiece = false;
            // } else {
            //     motorTime = 0;
            // }
        }
    }

    public void setIntakeVolts(double volts) {
        intakeMotor.setVoltage(-volts);
    }

    public double getIntakeCurrent() {
        return intakeMotor.getOutputCurrent();  
    }
    
    public void Grab(boolean forward) {
        velocity = intakeMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Intake Velocity", velocity);
        this.forward = forward;
        if (forward) {
            setIntakeVolts(Constants.IntakeVoltage); //positive intake voltage is outtake
        } else {
            setIntakeVolts(-1 * Constants.IntakeVoltage);
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
