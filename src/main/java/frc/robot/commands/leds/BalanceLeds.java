package frc.robot.commands.leds;

import java.util.function.DoubleSupplier;

import org.livoniawarriors.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LED_controller;

public class BalanceLeds extends CommandBase {
    //cals to edit
    final double MAX_ANGLE = 12;    //same both directions
    final double MAX_BRIGHT = 60;
    final double NUM_DIM_PIXELS = 4;

    //calculated value
    final double MIN_ANGLE = -MAX_ANGLE;

    ILedSubsystem leds;
    AddressableLEDBuffer m_ledBuffer;
    DoubleSupplier angle;

    public BalanceLeds(ILedSubsystem leds, DoubleSupplier angle) {
        this.leds = leds;
        this.angle = angle;
        addRequirements(leds);

        m_ledBuffer = new AddressableLEDBuffer(leds.getLength());
    }

    @Override
    public boolean runsWhenDisabled() { return true; }
    
    @Override
    public void initialize() { 
    }

    @Override
    public void execute() {
        //find the center led
        int center;
        int lastIndex = m_ledBuffer.getLength()-1;
        double value = angle.getAsDouble();
        if(value < MIN_ANGLE) {
            center = 0;
        } else if(value > MAX_ANGLE) {
            center = lastIndex;
        } else {
            center = (int)(((value - MIN_ANGLE)/(MAX_ANGLE - MIN_ANGLE)) * lastIndex);
        }

        //get the color for balance
        String mode = LED_controller.getLedMode();
        int hue;
        if(mode.equals(LED_controller.kPinkLeds)) {
            hue = LED_controller.PinkHue;
        } else {
            if(DriverStation.getAlliance() == Alliance.Red) {
                hue = LED_controller.RedHue;
            } else {
                hue = LED_controller.BlueHue;
            }
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
