package frc.robot;


import org.livoniawarriors.UtilFunctions;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.interfaces.IDriveControls;




public class LilMickeyDriveControls implements IDriveControls {
    private XboxController driveCont;
    private XboxController operCont;


    public LilMickeyDriveControls(){
        operCont = new XboxController(1);
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


    @Override
    public double GetArmShoulderPct() {
        if(driveCont.pov(0, null).getAsBoolean()) {
            return 0.3;
        } else if (driveCont.pov(180,null).getAsBoolean()) {
            return -0.3;
        } else {
            return 0.0;
        }
    }


    @Override
    public double GetArmElbowPct() {
        if(driveCont.pov(90, null).getAsBoolean()) {
            return -0.3;
        } else if (driveCont.pov(270,null).getAsBoolean()) {
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
    public JoystickButton TailUpRequested() {
        return new JoystickButton(driveCont, XboxController.Button.kY.value);
    }


    @Override
    public JoystickButton TailDownRequested() {
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


}
