package frc.robot;
import org.livoniawarriors.Logger;
import org.livoniawarriors.REVDigitBoard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IOperatorControls;
import frc.robot.interfaces.ISwerveDrive;

public class LED_controller {
    private static SerialPort sp;
    private REVDigitBoard digit;
    private boolean lastPieceMode;

    public LED_controller(){
        LED_controller.sp = new SerialPort(9600, Port.kOnboard);
        digit = new REVDigitBoard();
        lastPieceMode = false;
    }
    
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

    private int loopCounts;

    public void update(ISwerveDrive drive, Intake intake, Tail tail, IOperatorControls opControls) {
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
                } else if (Robot.getGamePieceMode() == Robot.CONE_MODE) {
                    //send cone mode
                    send("cone");
                } else {
                    send("cube");
                }
            }
        }

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
        var newMode = opControls.ChangePieceMode().getAsBoolean();
        boolean currentMode = Robot.getGamePieceMode();
        if(newMode == true && lastPieceMode == false) {
            Robot.setGamePieceMode(!currentMode);
        }

        String mode;
        if(currentMode == Robot.CONE_MODE) {
            mode = "Cone!";
        } else {
            mode = "Cube!";
        }
        SmartDashboard.putString("Piece Mode", mode);
        lastPieceMode = newMode;

        //check for rumble
        var rumble = 0.;
        if(intake.HasPiece() || tail.HasPiece()) {
            if(rumbleCounts < 40) {
                rumble = 0.6;
            }
            rumbleCounts++;
        } else {
            rumbleCounts = 0;
        }
        opControls.SetRumble(rumble);
    }
    int rumbleCounts;
}
