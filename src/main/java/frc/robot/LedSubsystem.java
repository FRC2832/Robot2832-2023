package frc.robot;

import org.livoniawarriors.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase implements ILedSubsystem {
    //we need ~1A of current per 15 LEDs (or 9W per 30LEDs)
    private static int LED_LENGTH = 5;
    private AddressableLED m_led;
    
    public LedSubsystem() {
        //initialize the string
        m_led = new AddressableLED(5);
        m_led.setLength(LED_LENGTH);

        //turn the string off in init
        AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
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
        return LED_LENGTH;
    }

    public void setData(AddressableLEDBuffer buffer) {
        m_led.setData(buffer);
    }
}
