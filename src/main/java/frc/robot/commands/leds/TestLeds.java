package frc.robot.commands.leds;

import org.livoniawarriors.ILedSubsystem;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestLeds extends CommandBase {
    ILedSubsystem leds;
    AddressableLEDBuffer m_ledBuffer;

    public TestLeds(ILedSubsystem leds) {
        this.leds = leds;
        addRequirements(leds);
        m_ledBuffer = new AddressableLEDBuffer(leds.getLength());

        //create a default record
        SmartDashboard.putNumber("TestLeds Index",-1);
        SmartDashboard.putNumber("TestLeds Hue",0);
        SmartDashboard.putNumber("TestLeds Sat",20);
        SmartDashboard.putNumber("TestLeds Value",0);
    }

    @Override
    public boolean runsWhenDisabled() { return true; }

    @Override
    public void initialize() { 
        
    }

    @Override
    public void execute() {

        int h = (int)SmartDashboard.getNumber("TestLeds Hue", 0);
        int s = (int)SmartDashboard.getNumber("TestLeds Sat", 20);
        int v = (int)SmartDashboard.getNumber("TestLeds Value", 0);

        //get color
        Color newColor = Color.fromHSV(h, s, v);
        Color fillColor;
        int pos = (int)SmartDashboard.getNumber("TestLeds Index", -1);

        //check if we are doing individual index
        if(pos >= 0){
            //fill the whole string with black, and set the specific color later
            fillColor = Color.kBlack;
        } else {
            fillColor = newColor;
        }

        //set all leds with the specified color
        for(int i=0; i<m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, fillColor);
        }

        //if we are coloring a specific LED, set it after it was cleared
        if(pos >= 0) {
            int max = leds.getLength()-1;
            if(pos > max)  pos = max;
            m_ledBuffer.setLED(pos, newColor);
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
