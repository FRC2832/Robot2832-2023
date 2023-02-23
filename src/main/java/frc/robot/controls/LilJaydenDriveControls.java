package frc.robot.controls;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.GrabberIntake;
import frc.robot.Intake;
import frc.robot.T16000M;
import frc.robot.interfaces.IDriveControls;


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
    public boolean BoostTriggerRequested() { //driver/operator will use this method
        return driveContRight.getTrigger();
    }

    @Override
    public boolean PrecisionTriggerRequested() { //driver/operator will use this method
        return driveContLeft.getTrigger();
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
    public void initializeButtons(Arm arm, Intake intake, GrabberIntake grabber) {
    }

    @Override
    public double GetPercentRightTriggerAxis() {
        if(driveContRight.getTrigger()){
            return 1.0;
        } else {
            return 0.0;
        }
    }

    @Override
    public double GetPercentLeftTriggerAxis() {
        if(driveContLeft.getTrigger()){
            return 1.0;
        } else {
            return 0.0;
        }
    }
}
