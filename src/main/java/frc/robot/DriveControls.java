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
        return driveCont.getLeftBumperPressed();
    }

    @Override
    public boolean PrecisionTriggerRequested() {
        return driveCont.getRightBumperPressed();
    }

    @Override
    public double GetArmAxis1Pct() {
        
        return -UtilFunctions.deadband(armCont.getyAxis1(), Constants.STICK_DEADBAND);
    }

    @Override
    public double GetArmAxis2Pct() {
        return -UtilFunctions.deadband(armCont.getyAxis1(), Constants.STICK_DEADBAND);
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

    
}
