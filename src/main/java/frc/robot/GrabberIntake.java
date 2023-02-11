package frc.robot;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class GrabberIntake implements Subsystem { 
    private CANSparkMax intakeMotor;
    private double nomVolt;
    private Timer timer;

    public GrabberIntake(){
        //grabber = new DoubleSolenoid(PneumaticsModuleType.REVPH,0,1);
        intakeMotor = new CANSparkMax(48,MotorType.kBrushless);
        intakeMotor.setInverted(true);
        timer = new Timer();
        
    }
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed%", intakeMotor.get());

    }

    public void Grab(boolean grabbing) {
        if (grabbing) {
            intakeMotor.setVoltage(Constants.IntakeVoltage);
        } else {
            intakeMotor.setVoltage(-1 * Constants.IntakeVoltage);
        }
        nomVolt = intakeMotor.getVoltageCompensationNominalVoltage();
        SmartDashboard.putNumber("Intake Nominal Voltage", nomVolt);
        
        if(nomVolt >= 0) {
            timer.start();
            if(timer.hasElapsed(2.0)) {
                intakeMotor.setVoltage(0.0);
            }
        }
        else {
            timer.stop();
            timer.reset();
        }
        
        
    }
}
