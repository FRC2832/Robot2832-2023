package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.interfaces.IOperatorControls;

public class LilJimmyDriveControls implements IOperatorControls {
    private XboxController operCont;

    public LilJimmyDriveControls(){
        operCont = new XboxController(2);
    }
   
    @Override
    public double GetArmKinXCommand() { //driver/operator will use this method
        if(operCont.getPOV() == 90){
            return 1.0;
        }
        else if(operCont.getPOV() == 270){
            return -1.0;
        }
        else{
            return 0.0;
        }
    }

    @Override
    public double GetArmKinZCommand() { //driver/operator will use this method
        if(operCont.getPOV() == 180){
            return 1.0;
        }
        else if(operCont.getPOV() == 0){
            return -1.0;
        }
        else{
            return 0.0;
        }
    }

    @Override
    public double GetArmShoulderPct() { //driver/operator will use this method
        if(operCont.getPOV() == 0) {
            return 0.3;
        } else if (operCont.getPOV() == 180) {
            return -0.3;
        } else {
            return 0.0;
        }
    }

    @Override
    public double GetArmElbowPct() { //driver/operator will use this method
        if(operCont.getPOV() == 90) {
            return -0.3;
        } else if (operCont.getPOV() == 270) {
            return 0.3;
        } else {
            return 0.0;
        }
    }

    @Override
    public Trigger ShoulderPosRequested() {
        return new Trigger(() -> operCont.getPOV() == 90);
    }

    @Override
    public Trigger ShoulderNegRequested() {
        return new Trigger(() -> operCont.getPOV() == 270);
    }

    @Override
    public Trigger ElbowPosRequested() {
        return new Trigger(() -> operCont.getPOV() == 0);
    }

    @Override
    public Trigger ElbowNegRequested() {
        return new Trigger(() -> operCont.getPOV() == 180);
    }

    @Override
    public JoystickButton ArmToPickupGroundCube(){
        return new JoystickButton(operCont, 16);
    }

    @Override
    public JoystickButton ArmToPickupGroundCone(){
        return new JoystickButton(operCont, XboxController.Button.kA.value);
    }
   
    @Override
    public JoystickButton ArmToPickupTail(){ //driver/operator will use this method
        return new JoystickButton(operCont, XboxController.Button.kRightBumper.value);
    }

    @Override
    public JoystickButton ArmToPickupHuman(){
        return new JoystickButton(operCont, 16);
    }
   
    @Override
    public JoystickButton ArmToSecureLocation(){
        return new JoystickButton(operCont, 16);
    }
   
    @Override
    public JoystickButton ArmToScoreLow(){ //driver/operator will use this method
        return new JoystickButton(operCont, 16);
    }
   
    @Override
    public JoystickButton ArmToScoreMiddle(){ //driver/operator will use this method
        return new JoystickButton(operCont, XboxController.Button.kB.value);
    }

    @Override
    public JoystickButton ArmToScoreMiddleFront(){ 
        return new JoystickButton(operCont, XboxController.Button.kX.value); 
    }
   
    @Override
    public JoystickButton ArmToScoreTop(){ //driver/operator will use this method
        return new JoystickButton(operCont, XboxController.Button.kY.value);
    }

    @Override
    public JoystickButton ArmToTransitionPoint(){
        return new JoystickButton(operCont, XboxController.Button.kLeftBumper.value);
    }

    @Override
    public Trigger GrabberUpRequested() { //driver/operator will use this method
        return new Trigger(() -> operCont.getRightY() > Constants.STICK_DEADBAND);
    }

    @Override
    public Trigger GrabberDownRequested() { //driver/operator will use this method
        return new Trigger(() -> operCont.getRightY() < -Constants.STICK_DEADBAND);
    }

    @Override
    public Trigger IntakeSuckRequested() { //driver/operator will use this method
        return new Trigger(() -> operCont.getRightTriggerAxis() > Constants.STICK_DEADBAND);
    }

    @Override
    public Trigger IntakeSpitRequested() { //driver/operator will use this method
        return new Trigger(() -> operCont.getLeftTriggerAxis() > Constants.STICK_DEADBAND);
    }

    @Override
    public Trigger ChangePieceMode() {
        return new Trigger(() -> operCont.getLeftStickButton());
    }
}

