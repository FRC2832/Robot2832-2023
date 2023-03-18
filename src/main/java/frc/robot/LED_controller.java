package frc.robot;

import org.livoniawarriors.Logger;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class LED_controller{
    private static SerialPort sp;
    private static boolean checkArm;
    private static boolean checkPiece;
    
    public LED_controller(){
        LED_controller.sp = new SerialPort(9600, Port.kOnboard);
    }
    
    public enum cmds {
        Arm(1),
        targeting_computer(2),
        brightness(3),
        cone(4),
        cube(5),
        kitt(6),
        lightning(7);
        //fault(8);
    
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
            case targeting_computer: 
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
            // case fault:
            //     cmd = "f";
            //     break;
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
            case targeting_computer: 
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
            // case fault:
            //     cmd = "f";
            //     break;
        }

        sp.writeString(cmd+Integer.toString(num)+"\r\n");
        return true;
    }

    public static void periodic(Arm arm, Intake intake){
        // if(Logger.isFaultSet()){
        //     send(cmds.fault);
        //     send something to ronyboard
        // }
        if(checkArm){
            send(cmds.lightning); //change the command later
        }
        if(checkPiece){
            send(cmds.lightning); //change the command later
        }
        
        if(Robot.getGamePieceMode() == Robot.CUBE_MODE){
            send(cmds.cube);
        }
        else{
            send(cmds.cone);
        }
    }

    public static void setArmAtPoint(boolean bool){
        checkArm = bool;
    }

    public static void setHasPiece(boolean bool){
        checkPiece = bool;
    }
}
