package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ArmBrakes extends SubsystemBase {
    private Solenoid shoulderBrake;
    private Solenoid elbowBrake;
    private SendableChooser<String> m_chooser;

    private final String BRAKES_AUTO = "Brakes Auto Apply";
    private final String BRAKES_OFF = "Brakes Off";
    private final String BRAKES_ON = "Brakes On";
    private String choice;
    
    public ArmBrakes(){
        super();
        shoulderBrake = new Solenoid(PneumaticsModuleType.REVPH,6);
        elbowBrake = new Solenoid(PneumaticsModuleType.REVPH,7);

        m_chooser = new SendableChooser<String>();
        m_chooser.setDefaultOption("Brakes Auto Apply", BRAKES_AUTO);
        m_chooser.addOption("Brakes Off", BRAKES_OFF);
        m_chooser.addOption("Brakes On", BRAKES_ON);
        
        SmartDashboard.putData("Brakes:", m_chooser);
    }

    @Override
    public void periodic() {
        choice = m_chooser.getSelected();
        if (choice == BRAKES_ON) {
            //false is fine here
            shoulderBrake.set(false);
            elbowBrake.set(false);
        } else if(choice == BRAKES_OFF) {
            shoulderBrake.set(true);
            elbowBrake.set(true);
        } else {
            //leave brakes alone
        }
    }

    public void Brake(boolean release, int channel) {
        //if we are in manual control, don't allow auto brake to happen
        if(choice != BRAKES_AUTO) { 
            return;
        }

        if (channel == 2) {
            if (release) {
                shoulderBrake.set(true);
            }
            else {
                shoulderBrake.set(false);
            }
        }
        else if (channel == 3) {
            if (release) {
                elbowBrake.set(true);
            }
            else {
                elbowBrake.set(false);
            }
        }
        else {
            SmartDashboard.putString("Error", "Invalid Braking Channel");
        }
        
    }
}
