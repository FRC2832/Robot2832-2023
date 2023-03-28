package frc.robot;
import org.livoniawarriors.Logger;
import org.livoniawarriors.REVDigitBoard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.interfaces.IOperatorControls;
import frc.robot.interfaces.ISwerveDrive;

public class LED_controller{
    private static SerialPort sp;
    private REVDigitBoard digit;

    public LED_controller(){
        LED_controller.sp = new SerialPort(9600, Port.kOnboard);
        digit = new REVDigitBoard();
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

    private boolean lastEnabled;

    public void update(ISwerveDrive drive, IOperatorControls opControls) {
        boolean enabled = DriverStation.isEnabled();

        if(!enabled) {
            send("prematch");
        }
        else if(DriverStation.isAutonomous()) {
            if(lastEnabled == false) {
                //first loop, say auto mode
                send("auton");
            } else {
                //send bubble mode
                send("bubble" + (int)drive.getPitch());
            }
        } else {
            //in teleop mode
            if(lastEnabled == false) {
                //first loop, say teleop mode
                send("teleop");
            } else if (opControls.IntakeSpitRequested().getAsBoolean()) {
                send("l");
            } else if (Robot.getGamePieceMode() == Robot.CONE_MODE) {
                //send cone mode
                send("cone");
            } else {
                send("cube");
            }
        }
        lastEnabled = enabled;

        //run the digit board
        if(Logger.FaultSet()) {
            digit.display(" FLT");
        } else if (Logger.StickyFaultSet()) {
            digit.display("SFLT");
        } else {
            digit.display("RONY");
        }
        if(digit.getButtonA()) {
            Logger.checkClearFaults(true);
        }
    }
}
