package frc.robot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class LED_controller{
    private SerialPort sp;
    public LED_controller(){
        this.sp = new SerialPort(9600, Port.kOnboard);

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

    this.sp.writeString(cmd+"/n");
    



    return true;
}
public boolean send(cmds prefix, int num){
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

    this.sp.writeString(cmd+Integer.toString(num)+"/n");
    return true;
}
}
