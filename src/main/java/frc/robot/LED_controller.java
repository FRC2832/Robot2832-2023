package frc.robot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class LED_controller{

    public LED_controller(){


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



public boolean send(cmds prefix){
    SerialPort  sp;
    sp = new SerialPort(9600, Port.kOnboard);
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



    return true;
}
public boolean send(cmds prefix, int num){
    SerialPort sp = new SerialPort(9600, Port.kOnboard);
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


    return true;
}
}
