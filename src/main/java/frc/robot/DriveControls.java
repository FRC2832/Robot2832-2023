package frc.robot;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmAutonPoint;
import frc.robot.commands.ArmManualOverride;
import frc.robot.commands.GrabberMove;
import frc.robot.commands.IntakeMove;
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
            return -0.3;
        } else if (armCont.getOrangeBottomRightButton()) {
            return 0.3;
        } else {
            return 0;
        }
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
        return new JoystickButton(armCont, 17);
    }

    @Override
    public JoystickButton GrabberDownRequested() {
        return new JoystickButton(armCont, 19);
    }

    @Override
    public JoystickButton GrabberSuckRequested() {
        return new JoystickButton(armCont, 18);
    }

    @Override
    public JoystickButton GrabberSpitRequested() {
        return new JoystickButton(armCont, 20);
    }

    @Override
    public double GetGrabberPct() {
        if(armCont.getRawButton(17)) {
            return 5.0; //if backwards switch negative
        } else if (armCont.getRawButton(19)) {
            return -5.0;
        } else {
            return 0.0;
        }
    }

    @Override
    public void initializeButtons(Arm arm, Intake intake, GrabberIntake grabber){
        ShoulderPosRequested().whileTrue(new ArmManualOverride(arm, this));
        ShoulderNegRequested().whileTrue(new ArmManualOverride(arm, this));
        ElbowPosRequested().whileTrue(new ArmManualOverride(arm, this));
        ElbowNegRequested().whileTrue(new ArmManualOverride(arm, this));
        ArmToPickupGround().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z));
        ArmToPickupTail().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));
        ArmToPickupHuman().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupHuman_X, Constants.ArmToPickupHuman_Z));
        ArmToSecureLocation().whileTrue(new ArmAutonPoint(arm, Constants.ArmToSecureLocation_X, Constants.ArmToSecureLocation_Z));
        ArmToScoreLow().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreLow_X, Constants.ArmToScoreLow_Z));
        ArmToScoreMiddle().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreMiddle_X, Constants.ArmToScoreMiddle_Z));
        ArmToScoreTop().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z));
        GrabberUpRequested().whileTrue(new IntakeMove(this, intake));
        GrabberDownRequested().whileTrue(new IntakeMove(this, intake));
        GrabberSuckRequested().whileTrue(new GrabberMove(this, grabber));
        GrabberSpitRequested().whileTrue(new GrabberMove(this, grabber));
    }

    @Override
    public JoystickButton ChangePieceMode() {
        return new JoystickButton(driveCont, XboxController.Button.kRightStick.value);        
    }
}
