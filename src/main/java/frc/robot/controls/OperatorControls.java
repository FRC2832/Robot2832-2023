package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Saitek;
import frc.robot.interfaces.IOperatorControls;
import edu.wpi.first.wpilibj.DriverStation;

public class OperatorControls implements IOperatorControls {
    private Saitek operCont;

    public OperatorControls() {
        operCont = new Saitek(2);
    }
   
    public double GetArmKinXCommand() {
        return operCont.getxAxis1();
    }

    public double GetArmKinZCommand() {
        return operCont.getyAxis1();
    }

    @Override
    public double GetArmShoulderPct() {
        if(operCont.getOrangeTopLeftButton()) {
            return 0.3;
        } else if (operCont.getOrangeBottomLeftButton()) {
            return -0.3;
        } else {
            return 0;
        }
    }

    @Override
    public double GetArmElbowPct() {
        if(operCont.getOrangeTopRightButton()) {
            return -0.3;
        } else if (operCont.getOrangeBottomRightButton()) {
            return 0.3;
        } else {
            return 0;
        }
    }

    @Override
    public Trigger ShoulderPosRequested() {
        return new Trigger(() -> operCont.getOrangeTopLeftButton());
    }

    @Override
    public Trigger ShoulderNegRequested() {
        return new Trigger(() -> operCont.getOrangeBottomLeftButton());   
    }

    @Override
    public Trigger ElbowPosRequested() {
        return new Trigger(() -> operCont.getOrangeTopRightButton());
    }
    @Override
    public JoystickButton ArmToPickupHuman_top(){
        return new JoystickButton(operCont, Saitek.Button.pinkBottomLeft.value);
    }
    @Override
    public JoystickButton ArmToPickupHuman_bottom(){
        return new JoystickButton(operCont, Saitek.Button.pinkBottomLeft.value);
    }
   


    @Override
    public Trigger ElbowNegRequested() {
        return new Trigger(() -> operCont.getOrangeBottomRightButton());
    }

    @Override
    public JoystickButton ArmToPickupGroundCube(){
        return new JoystickButton(operCont, Saitek.Button.yellowTopLeft.value);
    }

    @Override
    public JoystickButton ArmToPickupGroundCone(){
        return new JoystickButton(operCont, Saitek.Button.yellowTopLeft.value);
    }
   
    @Override
    public JoystickButton ArmToPickupTail(){
        return new JoystickButton(operCont, Saitek.Button.yellowBottomLeft.value);
    }

    // @Override
    // public JoystickButton ArmToPickupHuman(){
    //     return new JoystickButton(operCont, Saitek.Button.pinkBottomLeft.value);
    // }
   
    @Override
    public JoystickButton ArmToSecureLocation(){
        return new JoystickButton(operCont, Saitek.Button.yellowTopMiddle.value);
    }
   
    @Override
    public JoystickButton ArmToScoreLow(){
        return new JoystickButton(operCont, Saitek.Button.yellowBottomMiddle.value);
    }
   
    @Override
    public JoystickButton ArmToScoreMiddle(){
        return new JoystickButton(operCont, Saitek.Button.yellowBottomRight.value);
    }

    @Override
    public JoystickButton ArmToScoreMiddleFront(){ //driver/operator will use this method
        return new JoystickButton(operCont, Saitek.Button.pinkTopMiddle.value); 
    }
   
    @Override
    public JoystickButton ArmToScoreTop(){
        return new JoystickButton(operCont, Saitek.Button.yellowTopRight.value);
    }

    @Override
    public JoystickButton ArmToTransitionPoint(){
        return new JoystickButton(operCont, 21);
    }

    @Override
    public JoystickButton IntakeUpRequested() {
        return new JoystickButton(operCont, 17);
    }

    @Override
    public JoystickButton IntakeDownRequested() {
        return new JoystickButton(operCont, 19);
    }

    @Override
    public JoystickButton IntakeSuckRequested() {
        return new JoystickButton(operCont, 18);
    }

    @Override
    public JoystickButton IntakeSpitRequested() {
        return new JoystickButton(operCont, 20);
    }

    @Override
    public JoystickButton ChangePieceMode() {
        return new JoystickButton(operCont, Saitek.Button.pinkTopRight.value);        
    }

    public static boolean checkController() {
        if(DriverStation.getStickAxisCount(2) == 6) {
            if(DriverStation.getStickButtonCount(2) >= 10) {
                return true;
            }
        }
        return false;
    }
}
