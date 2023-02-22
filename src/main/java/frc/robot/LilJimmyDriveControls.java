package frc.robot;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmAutonPoint;
import frc.robot.commands.ChangeMode;
import frc.robot.commands.GrabberMove;
import frc.robot.commands.IntakeMove;
import frc.robot.interfaces.IDriveControls;


public class LilJimmyDriveControls implements IDriveControls {
    private XboxController driveCont;
    private XboxController operCont;

    public LilJimmyDriveControls(){
        driveCont = new XboxController(0);
        operCont = new XboxController(2);
    }
   
    @Override
    public boolean IsFieldOrientedResetRequested() {
        return driveCont.getLeftStickButtonPressed();
    }

    @Override
    public double GetXDrivePct() {
        return -UtilFunctions.deadband(driveCont.getLeftY(), Constants.STICK_DEADBAND);
    }

    @Override
    public double GetYDrivePct() {
        return -UtilFunctions.deadband(driveCont.getLeftX(), Constants.STICK_DEADBAND);
    }

    @Override
    public double GetTurnPct() {
        return -UtilFunctions.deadband(driveCont.getRightX(), Constants.STICK_DEADBAND);
    }

    @Override
    public boolean BoostTriggerRequested() {
        return driveCont.getRightTriggerAxis() > .1;
    }

    @Override
    public boolean PrecisionTriggerRequested() {
        return driveCont.getLeftTriggerAxis() > .1;
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
    public JoystickButton ShoulderPosRequested() {
        return new JoystickButton(driveCont, 8);
    }

    @Override
    public JoystickButton ShoulderNegRequested() {
        return new JoystickButton(driveCont, 8);
    }

    @Override
    public JoystickButton ElbowPosRequested() {
        return new JoystickButton(driveCont, 8);
    }

    @Override
    public JoystickButton ElbowNegRequested() {
        return new JoystickButton(driveCont, 8);
    }

    @Override
    public JoystickButton ArmToPickupGround(){
        return new JoystickButton(operCont, Saitek.Button.yellowTopLeft.value);
    }
   
    @Override
    public JoystickButton ArmToPickupTail(){ //driver/operator will use this method
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
    public JoystickButton ArmToScoreLow(){ //driver/operator will use this method
        return new JoystickButton(operCont, XboxController.Button.kA.value);
    }
   
    @Override
    public JoystickButton ArmToScoreMiddle(){ //driver/operator will use this method
        return new JoystickButton(operCont, XboxController.Button.kX.value);
    }
   
    @Override
    public JoystickButton ArmToScoreTop(){ //driver/operator will use this method
        return new JoystickButton(operCont, XboxController.Button.kY.value);
    }

    @Override
    public JoystickButton TailUpRequested() {
        return new JoystickButton(operCont, XboxController.Button.kRightBumper.value);
    }

    @Override
    public JoystickButton TailDownRequested() {
        return new JoystickButton(operCont, XboxController.Button.kLeftBumper.value);
    }

    @Override
    public JoystickButton GrabberUpRequested() { //driver/operator will use this method
        return new JoystickButton(operCont, XboxController.Axis.kRightY.value);
    }

    @Override
    public JoystickButton GrabberDownRequested() { //driver/operator will use this method
        return new JoystickButton(operCont, XboxController.Axis.kRightY.value);
    }

    @Override
    public double GetGrabberPct() { //driver/operator will use this method
        if(operCont.getRightY() > Constants.STICK_DEADBAND) {
            return -5.0 * operCont.getRightY(); //if backwards remove negative
        } else if (operCont.getRightY() < -Constants.STICK_DEADBAND) {
            return -5.0 * operCont.getRightY();
        } else {
            return 0.0;
        }
    }

    @Override
    public JoystickButton GrabberSuckRequested() { //driver/operator will use this method
        return new JoystickButton(operCont, XboxController.Axis.kRightTrigger.value);
    }

    @Override
    public JoystickButton GrabberSpitRequested() { //driver/operator will use this method
        return new JoystickButton(operCont, XboxController.Axis.kLeftTrigger.value);
    }

    @Override
    public void initializeButtons(Arm arm, Intake intake, GrabberIntake grabber) {
        ArmToPickupTail().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));
        ArmToScoreLow().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreLow_X, Constants.ArmToScoreLow_Z));
        ArmToScoreMiddle().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreMiddle_X, Constants.ArmToScoreMiddle_Z));
        ArmToScoreTop().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z));
        GrabberUpRequested().whileTrue(new IntakeMove(this, intake));
        GrabberDownRequested().whileTrue(new IntakeMove(this, intake));
        GrabberSuckRequested().whileTrue(new GrabberMove(this, grabber));
        GrabberSpitRequested().whileTrue(new GrabberMove(this, grabber));
        ChangePieceMode().toggleOnTrue(new ChangeMode());
    }

    @Override
    public JoystickButton ChangePieceMode() {
        return new JoystickButton(operCont, XboxController.Button.kRightStick.value);
    }
}

