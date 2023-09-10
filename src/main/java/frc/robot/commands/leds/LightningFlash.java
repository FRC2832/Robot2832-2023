package frc.robot.commands.leds;

import org.livoniawarriors.ILedSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LightningFlash extends SequentialCommandGroup {
    public LightningFlash(ILedSubsystem leds, int hue) {
        //Wpilib Hue is 0-180, FastLed was 0-360
        Color colorBright = Color.fromHSV(hue,255,255);
        Color colorDim = Color.fromHSV((int)MathUtil.inputModulus(hue-6, 0, 180),255,50);

        addCommands(
            new SolidColorLeds(leds, colorBright).withTimeout(0.1),
            new SolidColorLeds(leds, colorDim).withTimeout(0.1),
            new SolidColorLeds(leds, colorBright).withTimeout(0.05),
            new SolidColorLeds(leds, colorDim).withTimeout(0.05),
            new SolidColorLeds(leds, colorBright).withTimeout(0.05),
            new SolidColorLeds(leds, colorDim).withTimeout(0.05),
            new SolidColorLeds(leds, colorBright).withTimeout(0.1)
        );
    }
}