package frc.robot;

import org.livoniawarriors.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is really a subsection of the arm leds, so we handle that here
 */
public class TargetLedSubsystem extends SubsystemBase implements ILedSubsystem {
    private ArmLedSubsystem leds;
    
    public TargetLedSubsystem(ArmLedSubsystem leds) {
        this.leds = leds;
    }

    @Override
    public void periodic() {

    }

    public int getLength() {
        return ArmLedSubsystem.TARGET_LENGTH;
    }

    public void setData(AddressableLEDBuffer buffer) {
        leds.setTargetData(buffer);
    }

    public void setTargetData(AddressableLEDBuffer buffer) {
    }
}
