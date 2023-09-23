package frc.robot.commands.leds;

import org.livoniawarriors.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LED_controller;

public class ColorWave extends CommandBase {
    ILedSubsystem leds;
    AddressableLEDBuffer m_ledBuffer;
    UpdateValues hueCalc, valueCalc;
    String lastMode;

    public ColorWave(ILedSubsystem leds) {
        this.leds = leds;
        addRequirements(leds);
        m_ledBuffer = new AddressableLEDBuffer(leds.getLength());

        lastMode = "";
        hueCalc = new UpdateValues(-10, 10, 80);
        valueCalc = new UpdateValues(5, 60, 110);
    }

    @Override
    public boolean runsWhenDisabled() { return true; }
    
    @Override
    public void initialize() { 
    }

    @Override
    public void execute() {
        //check if there is a color change or first loop
        String curMode = LED_controller.getLedMode();
        if(lastMode != curMode) {
            int hue;
            if (curMode == LED_controller.kPinkLeds) {
                hue = LED_controller.PinkHue;
            } else {
                hue = LED_controller.GreenHue;
            }
            hueCalc = new UpdateValues(hue-5, hue+5, 80);
            valueCalc = new UpdateValues(5, 60, 110);
            lastMode = curMode;
        }

        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            double hue = hueCalc.get(i);
            double value = valueCalc.get(i);

            // Set the value
            m_ledBuffer.setHSV(i, (int)hue, 255, (int)value);
        }
        hueCalc.Step();
        valueCalc.Step();

        leds.setData(m_ledBuffer);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) { }

    private class UpdateValues {
        private double Min, Max;
        private int Count, loop;

        public UpdateValues(double min, double max, int count) {
            this.Min = min;
            this.Max = max;
            this.Count = count;
        }

        public double get(int step) {
            double diff = (Max - Min)/2;
            double mid = diff + Min;

            double value = mid + diff * Math.sin(2 * Math.PI * (loop + step) / Count);
            return value;
        }

        public void Step() {
            loop++;
            loop %= Count;
        }
    }
}
