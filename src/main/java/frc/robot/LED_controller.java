package frc.robot;
import edu.wpi.first.wpilibj.SerialPort;

public class DisplayManager(){

public enum cmds {
    Arm(1),
    targetting_computer(2),
    brightness(3),
    cone(4),
    cube(5),
    kitt(6),
    lightning(7);

    public final int value;
    Button(int value) {
      this.value = value;
}



public boolean send(cmds prefix,Optional<Integer> num){
    sp = SerialPort​(int 9600, SerialPort.Port "kOnboard");
    switch(prefix){
    case Arm:
        cmd ="a"
        break;
    case targetting_computer: 
        cmd ="t"
        break;
    case brightness:
        cmd ="b"
        break;
    case cone:
        cmd ="cone"
        break;
    case cube: 
        cmd ="cube"
        break;
    case kitt:
        cmd ="kitt"
        break;
    case lightning:
        cmd ="l"
        break;
    }

    if(num.isPresent() ){
        sp.writeString(cmd+Integer.toString(num)+"/n");
    }
    else{
    sp.writeString(cmd+"/n");
    }
    return true;
}
}
