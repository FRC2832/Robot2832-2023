package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ArmBrakes {
    private Solenoid shoulderBrake;
    private Solenoid elbowBrake;

    public ArmBrakes(){
        shoulderBrake = new Solenoid(PneumaticsModuleType.REVPH,2);
        elbowBrake = new Solenoid(PneumaticsModuleType.REVPH,3);
    }

    public void Brake(boolean release, int channel) {
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
