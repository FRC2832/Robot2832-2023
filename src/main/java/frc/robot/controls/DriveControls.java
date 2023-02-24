package frc.robot.controls;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Saitek;
import frc.robot.interfaces.IDriveControls;


public class DriveControls implements IDriveControls {
    private XboxController driveCont;
    private Saitek armCont;

    public DriveControls() {
        driveCont = new XboxController(0);
        armCont = new Saitek(2);
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
    public JoystickButton TailUpRequested() {
        return new JoystickButton(driveCont, XboxController.Button.kY.value);
    }

    @Override
    public JoystickButton TailDownRequested() {
        return new JoystickButton(driveCont, XboxController.Button.kX.value);
    }

    @Override
    public double GetPercentRightTriggerAxis() {
        return driveCont.getRightTriggerAxis();
    }

    @Override
    public double GetPercentLeftTriggerAxis() {
        return driveCont.getLeftTriggerAxis();
    }
}
