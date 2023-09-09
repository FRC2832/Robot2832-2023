package frc.robot.commands.leds;

import org.livoniawarriors.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LED_controller;
import frc.robot.Robot;

public class BreathLeds extends CommandBase {
    final double STEP_VALUE = 5;  //increments every 20ms
    final double MAX_VALUE = 200;
    final double MIN_VALUE = 25;

    ILedSubsystem leds;
    AddressableLEDBuffer m_ledBuffer;
    int breath;
    boolean increment;

    public BreathLeds(ILedSubsystem leds) {
        this.leds = leds;
        addRequirements(leds);
        m_ledBuffer = new AddressableLEDBuffer(leds.getLength());
    }

    @Override
    public boolean runsWhenDisabled() { return true; }
    
    @Override
    public void initialize() { 
        increment = true;
        breath = 125;
    }

    @Override
    public void execute() {
        Color color;

        if(increment) {
            breath += STEP_VALUE;
            if(breath >= MAX_VALUE) {
                increment = false;
            }
        } else {
            breath -= STEP_VALUE;
            if(breath <= MIN_VALUE) {
                increment = true;
            }
        }

        //figure out the light color
        if(Robot.getGamePieceMode() == Robot.CONE_MODE) {
            color = Color.fromHSV(LED_controller.ConeHue, 255, breath);
        } else {
            color = Color.fromHSV(LED_controller.CubeHue, 255, breath);
        }

        //set the pattern
        for(int i=0; i<m_ledBuffer.getLength(); i++) {
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
