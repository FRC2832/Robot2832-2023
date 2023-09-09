package frc.robot.commands.leds;

import org.livoniawarriors.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LED_controller;

public class TargetLeds extends CommandBase {
    //cals to edit
    final double MAX_BRIGHT = 60;
    final int NUM_DIM_PIXELS = 4;
    final int LOOP_DELAY = 4;    //how many loops before we change (1 loop = 20ms)

    ILedSubsystem leds;
    AddressableLEDBuffer m_ledBuffer;
    int center;
    boolean forward;
    int loopCount;

    public TargetLeds(ILedSubsystem leds) {
        this.leds = leds;
        addRequirements(leds);

        m_ledBuffer = new AddressableLEDBuffer(leds.getLength());
    }

    @Override
    public boolean runsWhenDisabled() { return true; }
    
    @Override
    public void initialize() { 
        center = m_ledBuffer.getLength()/2;
        forward = true;
        loopCount = 0;
    }

    @Override
    public void execute() {
        if(loopCount == 0) {
            //find the center led
            int lastIndex = m_ledBuffer.getLength()-1;
            if(forward) {
                center++;
                if(center >= lastIndex) {
                    forward = false;
                }
            } else {
                center--;
                if(center <= 0) {
                    forward = true;
                }
            }
        }
        loopCount = (loopCount+1) % LOOP_DELAY;

        //get the color for balance
        String mode = LED_controller.getLedMode();
        int hue;
        if(mode.equals(LED_controller.kPinkLeds)) {
            hue = LED_controller.PinkHue;
        } else {
            hue = LED_controller.GreenHue;
        }

        // set the strip
        for (int i=0; i<m_ledBuffer.getLength(); i++) {
            Color color = Color.kBlack;
            if((center-NUM_DIM_PIXELS <= i) && (i <= center+NUM_DIM_PIXELS)) {
                int bubble_bright = (int)(MAX_BRIGHT/(3*Math.abs(center-i) + 1));
                color = Color.fromHSV(hue, 255, bubble_bright);
            }

            m_ledBuffer.setLED(i, color);
        }
        leds.setData(m_ledBuffer);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) { }
}
