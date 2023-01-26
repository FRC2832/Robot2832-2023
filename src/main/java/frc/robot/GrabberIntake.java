package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class GrabberIntake implements Subsystem { 
    private DoubleSolenoid grabber;

    public GrabberIntake(){
        grabber = new DoubleSolenoid(PneumaticsModuleType.REVPH,0,1);
    }

    public void Grab(boolean grabbing) {
        if (grabbing) {
            grabber.set(Value.kForward);
        } else {
            grabber.set(Value.kReverse);
        }
    }
}
