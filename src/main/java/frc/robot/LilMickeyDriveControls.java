package frc.robot;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmManualOverride;
import frc.robot.interfaces.IDriveControls;


public class LilMickeyDriveControls implements IDriveControls {
    private XboxController driveCont;
    private XboxController operCont;

    public LilMickeyDriveControls(){
        driveCont = new XboxController(0);
    }
   
    @Override
    public boolean IsFieldOrientedResetRequested() { //driver/operator will use this method
        return driveCont.getLeftStickButtonPressed();
    }

    @Override
    public double GetXDrivePct() { //driver/operator will use this method
        return -UtilFunctions.deadband(driveCont.getLeftY(), Constants.STICK_DEADBAND);
    }

    @Override
    public double GetYDrivePct() { //driver/operator will use this method
        return -UtilFunctions.deadband(driveCont.getLeftX(), Constants.STICK_DEADBAND);
    }

    @Override
    public double GetTurnPct() { //driver/operator will use this method
        return -UtilFunctions.deadband(driveCont.getRightX(), Constants.STICK_DEADBAND);
    }

    @Override
    public boolean BoostTriggerRequested() { //driver/operator will use this method
        return driveCont.getRightTriggerAxis() > .1;
    }

    @Override
    public boolean PrecisionTriggerRequested() { //driver/operator will use this method
        return driveCont.getLeftTriggerAxis() > .1;
    }

    @Override
    public double GetArmKinXCommand() {
        if(operCont.pov(90, null).getAsBoolean()){
            return 1.0;
        }
        else if(operCont.pov(270, null).getAsBoolean()){
            return -1.0;
        }
        else{
            return 0.0;
        }
    }

    @Override
    public double GetArmKinZCommand() {
        if(operCont.pov(180, null).getAsBoolean()){
            return 1.0;
        }
        else if(operCont.pov(0, null).getAsBoolean()){
            return -1.0;
        }
        else{
            return 0.0;
        }
    }

    //"Give it to Mickey" -James
    @Override
    public double GetArmShoulderPct() { //driver/operator will use this method
        if(driveCont.pov(0, null).getAsBoolean()) {
            return 0.3;
        } else if (driveCont.pov(180,null).getAsBoolean()) {
            return -0.3;
        } else {
            return 0.0;
        }
    }

    @Override
    public double GetArmElbowPct() { //driver/operator will use this method
        if(driveCont.pov(90, null).getAsBoolean()) {
            return -0.3;
        } else if (driveCont.pov(270,null).getAsBoolean()) {
            return 0.3;
        } else {
            return 0.0;
        }
    }

    @Override
    public JoystickButton ShoulderPosRequested() { //driver/operator will use this method
        return new JoystickButton(driveCont, 8);
    }

    @Override
    public JoystickButton ShoulderNegRequested() { //driver/operator will use this method
        return new JoystickButton(driveCont, 8);
    }

    @Override
    public JoystickButton ElbowPosRequested() { //driver/operator will use this method
        return new JoystickButton(driveCont, 8);
    }

    @Override
    public JoystickButton ElbowNegRequested() { //driver/operator will use this method
        return new JoystickButton(driveCont, 8);
    }

    @Override
    public JoystickButton ArmToPickupGround(){
        return new JoystickButton(operCont, Saitek.Button.yellowTopLeft.value);
    }
   
    @Override
    public JoystickButton ArmToPickupTail(){
        return new JoystickButton(operCont, XboxController.Button.kB.value);
    }

    @Override
    public JoystickButton ArmToPickupHuman(){
        return new JoystickButton(operCont, Saitek.Button.pinkBottomLeft.value);
    }
   
    @Override
    public JoystickButton ArmToSecureLocation(){
        return new JoystickButton(operCont, Saitek.Button.yellowTopMiddle.value);
    }
   
    @Override
    public JoystickButton ArmToScoreLow(){
        return new JoystickButton(operCont, XboxController.Button.kA.value);
    }
   
    @Override
    public JoystickButton ArmToScoreMiddle(){
        return new JoystickButton(operCont, XboxController.Button.kX.value);
    }

    @Override
    public JoystickButton ArmToScoreTop(){
        return new JoystickButton(operCont, XboxController.Button.kY.value);
    }

    @Override
    public JoystickButton TailUpRequested() { //driver/operator will use this method
        return new JoystickButton(driveCont, XboxController.Button.kY.value);
    }

    @Override
    public JoystickButton TailDownRequested() { //driver/operator will use this method
        return new JoystickButton(driveCont, XboxController.Button.kX.value);
    }

    @Override
    public JoystickButton GrabberUpRequested() {
        return new JoystickButton(operCont, XboxController.Axis.kRightY.value);
    }

    @Override
    public JoystickButton GrabberDownRequested() {
        return new JoystickButton(operCont, XboxController.Axis.kRightY.value);
    }

    @Override
    public double GetGrabberPct() {
        if(operCont.getRightY() > Constants.STICK_DEADBAND) {
            return -5.0 * operCont.getRightY(); //if backwards remove negative
        } else if (operCont.getRightY() < -Constants.STICK_DEADBAND) {
            return -5.0 * operCont.getRightY();
        } else {
            return 0.0;
        }
    }

    @Override
    public JoystickButton GrabberSuckRequested() {
        return new JoystickButton(operCont, XboxController.Axis.kRightTrigger.value);
    }

    @Override
    public JoystickButton GrabberSpitRequested() {
        return new JoystickButton(operCont, XboxController.Axis.kLeftTrigger.value);
    }

    @Override
    public void initializeButtons(Arm arm, Intake intake, GrabberIntake grabber) {
        ShoulderPosRequested().whileTrue(new ArmManualOverride(arm, this));
        ShoulderNegRequested().whileTrue(new ArmManualOverride(arm, this));
        ElbowPosRequested().whileTrue(new ArmManualOverride(arm, this));
        ElbowNegRequested().whileTrue(new ArmManualOverride(arm, this));
    }

    @Override
    public JoystickButton ChangePieceMode() {
        return new JoystickButton(operCont, XboxController.Button.kRightStick.value);
    }
}
