package frc.robot.controls;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.GrabberIntake;
import frc.robot.Intake;
import frc.robot.interfaces.IDriveControls;


public class LilMickeyDriveControls implements IDriveControls {
    private XboxController driveCont;

    public LilMickeyDriveControls(){
        driveCont = new XboxController(0);
        
        SmartDashboard.putBoolean("Field Oriented", false);
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
    public JoystickButton TailUpRequested() { 
        return new JoystickButton(driveCont, XboxController.Button.kY.value);
    }

    @Override
    public JoystickButton TailDownRequested() { 
        return new JoystickButton(driveCont, XboxController.Button.kX.value);
    }

    @Override
    public void initializeButtons(Arm arm, Intake intake, GrabberIntake grabber) {
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