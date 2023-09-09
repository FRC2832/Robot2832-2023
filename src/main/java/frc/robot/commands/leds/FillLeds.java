package frc.robot.commands.leds;

import org.livoniawarriors.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LED_controller;
import frc.robot.Robot;

public class FillLeds extends CommandBase {
    final double STEP_TIME = 0.04;  //increment every 40ms
    ILedSubsystem leds;
    AddressableLEDBuffer m_ledBuffer;
    double startTime;
    int currentLight;

    public FillLeds(ILedSubsystem leds) {
        this.leds = leds;
        addRequirements(leds);
        m_ledBuffer = new AddressableLEDBuffer(leds.getLength());
    }

    @Override
    public boolean runsWhenDisabled() { return true; }
    
    @Override
    public void initialize() { 
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        Color color;

        //calculate the current light
        double time = Timer.getFPGATimestamp() - startTime;
        currentLight = (int)(time/STEP_TIME);

        //figure out the light color
        if(Robot.getGamePieceMode() == Robot.CONE_MODE) {
            color = Color.fromHSV(LED_controller.ConeHue, 255, 125);
        } else {
            color = Color.fromHSV(LED_controller.CubeHue, 255, 125);
        }

        //set the pattern
        for(int i=0; i<m_ledBuffer.getLength(); i++) {
            if(i > currentLight) {
                color = Color.kBlack;
            }
            m_ledBuffer.setLED(i, color);
        }
        leds.setData(m_ledBuffer);
    }

    @Override
    public boolean isFinished() {
        return currentLight > m_ledBuffer.getLength();
    }

    @Override
    public void end(boolean interrupted) { }
}
