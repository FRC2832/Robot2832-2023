package frc.robot;
import org.livoniawarriors.Logger;
import org.livoniawarriors.REVDigitBoard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.leds.RainbowLeds;
import frc.robot.commands.leds.TestLeds;
import frc.robot.interfaces.IDriveControls;
import frc.robot.interfaces.IOperatorControls;
import frc.robot.interfaces.ISwerveDrive;

public class LED_controller {
    private static SerialPort sp;
    private REVDigitBoard digit;
    private boolean lastPieceMode;
    private int loopCounts;
    private int rumbleCounts;
    private LedSubsystem leds;

    public LED_controller(){
        LED_controller.sp = new SerialPort(9600, Port.kOnboard);
        digit = new REVDigitBoard();
        lastPieceMode = false;

        //new LED controller
        leds = new LedSubsystem();
        var defaultCmd = new RainbowLeds(leds);
        leds.setDefaultCommand(defaultCmd);
        defaultCmd.schedule();
        
        //put button to try leds on the dashboard
        SmartDashboard.putData(new TestLeds(leds));
    }

    /*
        Operating modes:
        1.  "prematch" - Default on powerup is "prematch".   This shows the sound reactive arm lights and puts a boombox animation on the LED panel.  At end of match, if we want to go back to this, send "prematch"
        2.  "auton" - send to trigger the flash drive driver animation.  Puts the arm lights in balance mode.  See below for bubble command.
        3.  "teleop" - send to put the robot in normal LED operating mode.  LED panel scrolls through sponsors and animations.   Arm lights react to "cone" and "cube" commands to change color.  Arm lights automatically build up energy and pulse when maxed out.
        After match, send "prematch" to put us back in sound reactive mode.

        Discrete commands:
        "prematch" :  Puts us in prematch mode (default node on startup)
        "auton" - Puts us in autonomous mode
        "teleop" - Puts us in teleop mode
        "bubbleX" - Displays the bubble animation on the arms during charge dock balancing.   X is in the range from -12 to 12.  0 is when balanced.
        "cone" - Set arm lights to yellow in teleop
        "cube" - set arm lights to purple in teleop
        "bX" - sets overall brightness to value X.  Range 0-255
        "tX" - sets targeting computer to turn on light X.   If 0, performs sweep animation
        "kitt" - turns targeting computer to all red.   "t0" puts it back to rainbow colors
        "l" (lower case letter L) - Triggers the lightning animation
        "aX" - Turns on X number of arm lights, starting at base.   a0 allows automatic increment
        "red" - turn lights red
        "blue" - turn lights blue
    */
    
    public enum cmds {
        Arm(1),
        targetting_computer(2),
        brightness(3),
        cone(4),
        cube(5),
        kitt(6),
        lightning(7);
    
        public final int value;
        cmds(int value) {
            this.value = value;
        }
    }

    public static boolean send(cmds prefix){
        String cmd = "";
        switch(prefix){
        case Arm:
            cmd ="a";
            break;
        case targetting_computer: 
            cmd ="t";
            break;
        case brightness:
            cmd ="b";
            break;
        case cone:
            cmd ="cone";
            break;
        case cube: 
            cmd ="cube";
            break;
        case kitt:
            cmd ="kitt";
            break;
        case lightning:
            cmd ="l";
            break;
        }

        sp.writeString(cmd+"\r\n");
        return true;
    }

    public static boolean send(cmds prefix, int num){
        String cmd = "";
        switch(prefix){
        case Arm:
            cmd ="a";
            break;
        case targetting_computer: 
            cmd ="t";
            break;
        case brightness:
            cmd ="b";
            break;
        case cone:
            cmd ="cone";
            break;
        case cube: 
            cmd ="cube";
            break;
        case kitt:
            cmd ="kitt";
            break;
        case lightning:
            cmd ="l";
            break;
        }

        sp.writeString(cmd+Integer.toString(num)+"\r\n");
        return true;
    }

    private void send(String msg) {
        sp.writeString(msg+"\r\n");
    }

    public void update(ISwerveDrive drive, Intake intake, Tail tail, IDriveControls driver, IOperatorControls opControls) {
        //check for rumble
        var rumble = 0.;
        if(intake.HasPiece() || tail.HasPiece()) {
            if(rumbleCounts < 40) {
                rumble = 0.8;
            }
            rumbleCounts++;
        } else {
            rumbleCounts = 0;
        }
        driver.SetRumble(rumble);
        opControls.SetRumble(rumble);

        if(loopCounts % 50 == 0) {
            if(!DriverStation.isEnabled()) {
                send("prematch");
            } else if(DriverStation.isAutonomous()) {
                send("auton");
            } else {
                send("teleop");
            }
        } else if(loopCounts % 5 == 3) {
            if(!DriverStation.isEnabled()) {
                //do nothing in disabled
            }
            else if(DriverStation.isAutonomous()) {
                //send bubble mode
                var pitch = (int)(drive.getPitch() + 12);
                if(pitch < 1) {
                    pitch = 1;
                }
                if (pitch > 25) {
                    pitch = 25;
                }
                send("bubble" + pitch);
            } else {
                if (opControls.IntakeSpitRequested().getAsBoolean()) {
                    send("l");
                } else if (rumble > 0.1) {
                    if(DriverStation.getAlliance() == Alliance.Red) {
                        send("red");
                    } else {
                        send("blue");
                    }
                } else if (Robot.getGamePieceMode() == Robot.CONE_MODE) {
                    //send cone mode
                    send("cone");
                } else {
                    send("cube");
                }
            }
        }
        loopCounts++;

        //run the digit board
        if(Logger.FaultSet()) {
            digit.display(" FLT");
        } else if (Logger.StickyFaultSet()) {
            digit.display("SFLT");
        } else {
            digit.display("RONY");
        }
        /*
        if(digit.getButtonA()) {
            Logger.checkClearFaults(true);
        } */

        //check if piece mode needs to be switched
        String mode;
        var newMode = opControls.ChangePieceMode().getAsBoolean();
        boolean currentMode = Robot.getGamePieceMode();
        if(newMode == true && lastPieceMode == false) {
            Robot.setGamePieceMode(!currentMode);
        }
        if(currentMode == Robot.CONE_MODE) {
            mode = "Cone!";
        } else {
            mode = "Cube!";
        }
        SmartDashboard.putString("Piece Mode", mode);
        lastPieceMode = newMode;
    }
}
