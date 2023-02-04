package frc.robot;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.interfaces.IDriveControls;


public class DriveControls implements IDriveControls {
    private XboxController driveCont;
    private Saitek armCont;

    public DriveControls() {
        driveCont = new XboxController(0);
        armCont = new Saitek(1);
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
            return 0.3;
        } else if (armCont.getOrangeBottomRightButton()) {
            return -0.3;
        } else {
            return 0;
        }
    }

    @Override
    public boolean IntakeConeRequested() {
        return armCont.getYellowTopLeftButton();
    }

    @Override
    public boolean OuttakeConeRequested() {
        return armCont.getYellowTopMiddleButton();
    }

    @Override
    public JoystickButton CubeGrabCloseRequested() {
        return new JoystickButton(armCont, Saitek.Button.pinkTopLeft.value);
    }

    @Override
    public JoystickButton CubeGrabOpenRequested() {
        return new JoystickButton(armCont, Saitek.Button.pinkTopMiddle.value);
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

}
