package frc.robot;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.interfaces.IDriveControls;


public class LilJaydenDriveControls implements IDriveControls {
    private T16000M driveContLeft;
    private T16000M driveContRight;
    private Saitek armCont;

    public LilJaydenDriveControls() {
        driveContLeft = new T16000M(0);
        driveContRight = new T16000M(1);
    }
   
    @Override
    public boolean IsFieldOrientedResetRequested() { //driver/operator will use this method
        return driveContLeft.getMiddlePressed(); //TODO: make sure this is the thumb press down
    }

    @Override
    public double GetXDrivePct() { //driver/operator will use this method
        return -UtilFunctions.deadband(driveContLeft.getxAxis1(), Constants.STICK_DEADBAND);
    }

    @Override
    public double GetYDrivePct() { //driver/operator will use this method
        return -UtilFunctions.deadband(driveContLeft.getyAxis1(), Constants.STICK_DEADBAND);
    }

    @Override
    public double GetTurnPct() { //driver/operator will use this method
        return -UtilFunctions.deadband(driveContRight.getxAxis1(), Constants.STICK_DEADBAND);
    }

    @Override
    public boolean BoostTriggerRequested() { //driver/operator will use this method
        return driveContRight.getTrigger();
    }

    @Override
    public boolean PrecisionTriggerRequested() { //driver/operator will use this method
        return driveContLeft.getTrigger();
    }

    public double GetArmKinXCommand() {
        return armCont.getxAxis1();
    }

    public double GetArmKinZCommand() {
        return armCont.getyAxis1();
    }

    @Override
    public double GetArmShoulderPct() {
        if(armCont.getOrangeTopLeftButton()) {
            return 0.3;
        } else if (armCont.getOrangeBottomLeftButton()) {
            return -0.3;
        } else {
            return 0;
        }
    }

    @Override
    public double GetArmElbowPct() {
        if(armCont.getOrangeTopRightButton()) {
            return -0.3;
        } else if (armCont.getOrangeBottomRightButton()) {
            return 0.3;
        } else {
            return 0;
        }
    }

    @Override
    public JoystickButton ShoulderPosRequested() {
        return new JoystickButton(armCont, Saitek.Button.orangeTopLeft.value);
    }

    @Override
    public JoystickButton ShoulderNegRequested() {
        return new JoystickButton(armCont, Saitek.Button.orangeBottomLeft.value);
    }

    @Override
    public JoystickButton ElbowPosRequested() {
        return new JoystickButton(armCont, Saitek.Button.orangeTopRight.value);
    }

    @Override
    public JoystickButton ElbowNegRequested() {
        return new JoystickButton(armCont, Saitek.Button.orangeBottomRight.value);
    }

    @Override
    public JoystickButton ArmToPickupGround(){
        return new JoystickButton(armCont, Saitek.Button.yellowTopLeft.value);
    }
   
    @Override
    public JoystickButton ArmToPickupTail(){
        return new JoystickButton(armCont, Saitek.Button.yellowBottomLeft.value);
    }

    @Override
    public JoystickButton ArmToPickupHuman(){
        return new JoystickButton(armCont, Saitek.Button.pinkBottomLeft.value);
    }
   
    @Override
    public JoystickButton ArmToSecureLocation(){
        return new JoystickButton(armCont, Saitek.Button.yellowTopMiddle.value);
    }
   
    @Override
    public JoystickButton ArmToScoreLow(){
        return new JoystickButton(armCont, Saitek.Button.yellowBottomMiddle.value);
    }
   
    @Override
    public JoystickButton ArmToScoreMiddle(){
        return new JoystickButton(armCont, Saitek.Button.yellowBottomRight.value);
    }
   
    @Override
    public JoystickButton ArmToScoreTop(){
        return new JoystickButton(armCont, Saitek.Button.yellowTopRight.value);
    }

    @Override
    public JoystickButton TailUpRequested() { //driver/operator will use this method
        return new JoystickButton(driveContRight, T16000M.Button.middle.value);
    }

    @Override
    public JoystickButton TailDownRequested() {  //driver/operator will use this method
        return new JoystickButton(driveContRight, T16000M.Button.middle.value);
    }

    @Override
    public JoystickButton GrabberUpRequested() {
        return new JoystickButton(armCont, 17);
    }

    @Override
    public JoystickButton GrabberDownRequested() {
        return new JoystickButton(armCont, 19);
    }

    @Override
    public JoystickButton GrabberSuckRequested() {
        return new JoystickButton(armCont, 18);
    }

    @Override
    public JoystickButton GrabberSpitRequested() {
        return new JoystickButton(armCont, 20);
    }

    @Override
    public double GetGrabberPct() {
        if(armCont.getRawButton(17)) {
            return 5.0; //if backwards switch negative
        } else if (armCont.getRawButton(19)) {
            return -5.0;
        } else {
            return 0.0;
        }
    }

    @Override
    public void initializeButtons(Arm arm, Intake intake, GrabberIntake grabber) {
        // nothing needed here yet
    }
}
