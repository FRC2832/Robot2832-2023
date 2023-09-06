package org.livoniawarriors;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ILedSubsystem extends Subsystem {
    int getLength();
    void setData(AddressableLEDBuffer buffer);
}
