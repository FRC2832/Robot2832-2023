package frc.robot.controls;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.T16000M;
import frc.robot.interfaces.IDriveControls;
import edu.wpi.first.wpilibj.DriverStation;


public class LilJaydenDriveControls implements IDriveControls {
    private T16000M driveContLeft;
    private T16000M driveContRight;

    public LilJaydenDriveControls() {
        driveContLeft = new T16000M(0);
        driveContRight = new T16000M(1);
        
        SmartDashboard.putBoolean("Field Oriented", true);
    }
   
    @Override
    public boolean IsFieldOrientedResetRequested() { //driver/operator will use this method
        return driveContLeft.getRawButtonPressed(2); 
    }

    @Override
    public double GetXDrivePct() { //driver/operator will use this method
        return -UtilFunctions.deadband(driveContRight.getyAxis1(), Constants.STICK_DEADBAND);
    }

    @Override
    public double GetYDrivePct() { //driver/operator will use this method
        return -UtilFunctions.deadband(driveContRight.getxAxis1(), Constants.STICK_DEADBAND);
    }

    @Override
    public double GetTurnPct() { //driver/operator will use this method
        return -UtilFunctions.deadband(driveContLeft.getxAxis1(), Constants.STICK_DEADBAND);
    }

    @Override
    public JoystickButton TailUpRequested() { 
        return new JoystickButton(driveContRight, T16000M.Button.left.value);
    }

    @Override
    public JoystickButton TailDownRequested() {  //driver/operator will use this method
        return new JoystickButton(driveContRight, T16000M.Button.middle.value);
    }

    @Override
    public JoystickButton TailStowRequested() {
        return new JoystickButton(driveContRight, T16000M.Button.right.value);
    }

    @Override
    public JoystickButton TailHumanRequested() {
        return new JoystickButton(driveContLeft, T16000M.Button.right.value);
    }

    @Override
    public double GetBoostTriggerRequest() {
        if(driveContRight.getTrigger()){
            return 1.0;
        } else {
            return 0.0;
        }
    }

    @Override
    public double GetPrecisionTriggerRequest() {
        if(driveContLeft.getTrigger()){
            return 1.0;
        } else {
            return 0.0;
        }
    }

    public static boolean checkController() {
        if(DriverStation.getStickAxisCount(0) == 4 && DriverStation.getStickAxisCount(1) == 4) {
            if(DriverStation.getStickButtonCount(0) == 16 && DriverStation.getStickButtonCount(1) == 16) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void SetRumble(double pct) {
    }
}
