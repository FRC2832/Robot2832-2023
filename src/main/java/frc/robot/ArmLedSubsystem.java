package frc.robot;

import org.livoniawarriors.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmLedSubsystem extends SubsystemBase implements ILedSubsystem {
    //we need ~1A of current per 15 LEDs (or 9W per 30LEDs)
    private static int ARM_LENGTH = 47;
    public static int TARGET_LENGTH = 27;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    
    public ArmLedSubsystem() {
        //initialize the string
        m_led = new AddressableLED(1);
        m_led.setLength(ARM_LENGTH + TARGET_LENGTH);

        //turn the string off in init
        m_ledBuffer = new AddressableLEDBuffer(ARM_LENGTH + TARGET_LENGTH);
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, Color.kBlack);
        }

        // Set the initial state
        m_led.setData(m_ledBuffer);

        //start the string
        m_led.start();
    }

    @Override
    public void periodic() {

    }

    public int getLength() {
        return ARM_LENGTH;
    }

    public void setData(AddressableLEDBuffer buffer) {
        //copy the data into the buffer
        for(int i=0; i<buffer.getLength() && i<m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, buffer.getLED(i));
        }

        //send it
        m_led.setData(m_ledBuffer);
    }

    public void setTargetData(AddressableLEDBuffer buffer) {
        //copy the data into the buffer
        for(int i=0; i<buffer.getLength() && i<(m_ledBuffer.getLength()+ARM_LENGTH); i++) {
            m_ledBuffer.setLED(i+ARM_LENGTH, buffer.getLED(i));
        }
        
        //send it
        m_led.setData(m_ledBuffer);
    }
}
