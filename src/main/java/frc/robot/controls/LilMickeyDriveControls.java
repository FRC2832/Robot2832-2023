package frc.robot.controls;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.interfaces.IDriveControls;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


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
    public JoystickButton TailUpRequested() { 
        return new JoystickButton(driveCont, XboxController.Button.kRightBumper.value);
    }

    @Override
    public JoystickButton TailDownRequested() { 
        return new JoystickButton(driveCont, XboxController.Button.kLeftBumper.value);
    }
    
    @Override
    public double GetPrecisionTriggerRequest() {
        return driveCont.getRightTriggerAxis();
    }

    @Override
    public double GetBoostTriggerRequest() {
        return driveCont.getLeftTriggerAxis();
    }

    public static boolean checkController() {
        if(DriverStation.getStickAxisCount(0) == 6) {
            if(DriverStation.getStickButtonCount(0) >= 10) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void setBothRumble(double val){
        driveCont.setRumble(RumbleType.kBothRumble, val);
    }

    @Override
    public void setLeftRumble(double val){
        driveCont.setRumble(RumbleType.kLeftRumble, val);
    }

    @Override
    public void setRightRumble(double val){
        driveCont.setRumble(RumbleType.kRightRumble, val);
    }
}
