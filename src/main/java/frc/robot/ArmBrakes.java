package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ArmBrakes {
    private Solenoid shoulderBrake;
    private Solenoid elbowBrake;

    public ArmBrakes(){
        shoulderBrake = new Solenoid(PneumaticsModuleType.REVPH,0);
        elbowBrake = new Solenoid(PneumaticsModuleType.REVPH,1);
    }

    public void Brake(boolean release, int channel) {
        if (channel == 0) {
            if (release) {
                shoulderBrake.set(true);
            }
            else {
                shoulderBrake.set(false);
            }
        }
        else if (channel == 1) {
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
