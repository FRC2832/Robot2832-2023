package frc.robot;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmAutonPoint;
import frc.robot.commands.ArmManualOverride;
import frc.robot.commands.GrabberMove;
import frc.robot.commands.IntakeMove;
import frc.robot.interfaces.IDriveControls;


public class LilHaydenDriveControls implements IDriveControls {
    private XboxController driveCont;
    private Saitek armCont;

    public LilHaydenDriveControls() {
        armCont = new Saitek(2); //WARNING: HE USES THE CONTROLLER UPSIDE DOWN; port 2 since Jayden has 2 controllers technically
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
    public double GetArmShoulderPct() { //driver/operator will use this method
        if(armCont.getyAxis1() > 0) { //if he moves controller up, y goes towards 1 not -1
            return 0.3 * armCont.getyAxis1();//positive #
        } else if (armCont.getyAxis1() < 0) { //if he moves controller down, y towards -1
            return 0.3 * armCont.getyAxis1();//negative #
        } else if(armCont.getRawButton(10)) {
            return 0.3;
        } else if (armCont.getRawButton(5)) {
            return -0.3;
        } else {
            return 0;
        }
    }

    @Override
    public double GetArmElbowPct() {  //driver/operator will use this method
        if(armCont.getxAxis1() > 0) { //if he moves controller left, y goes towards 1 not -1
            return 0.3 * armCont.getxAxis1();//positive #
        } else if (armCont.getxAxis1() < 0) { //if he moves controller right, y towards -1
            return 0.3 * armCont.getxAxis1();//negative #
        } else if(armCont.getRawButton(9)) {
            return -0.3;
        } else if (armCont.getRawButton(4)) {
            return 0.3;
        } else {
            return 0;
        }
    }

    @Override
    public JoystickButton ShoulderPosRequested() { //driver/operator will use this method
        return new JoystickButton(armCont, 10);
    }

    @Override
    public JoystickButton ShoulderNegRequested() { //driver/operator will use this method
        return new JoystickButton(armCont, 5);
    }

    @Override
    public JoystickButton ElbowPosRequested() { //driver/operator will use this method
        return new JoystickButton(armCont, 9);
    }

    @Override
    public JoystickButton ElbowNegRequested() { //driver/operator will use this method
        return new JoystickButton(armCont, 4);
    }

    @Override
    public JoystickButton ArmToPickupGround(){ //driver/operator will use this method
        return new JoystickButton(armCont, 18); //TODO: take stevens point
    }
   
    @Override
    public JoystickButton ArmToPickupTail(){ //driver/operator will use this method
        return new JoystickButton(armCont, 20);
    }

    @Override
    public JoystickButton ArmToPickupHuman(){ //driver/operator will use this method
        return new JoystickButton(armCont, 19);
    }
   
    @Override
    public JoystickButton ArmToSecureLocation(){ //driver/operator will use this method
        return new JoystickButton(armCont, 21);
    }
   
    @Override
    public JoystickButton ArmToScoreLow(){ //driver/operator will use this method
        return new JoystickButton(armCont, 6); //technically he wanted this button to be for cubes, 12 for cones
    }
   
    @Override
    public JoystickButton ArmToScoreMiddle(){ //driver/operator will use this method
        return new JoystickButton(armCont, 7); //technically he wanted this button to be for cubes, 14 for cones
    }
   
    @Override
    public JoystickButton ArmToScoreTop(){ //driver/operator will use this method
        return new JoystickButton(armCont, 8); //technically he wanted this button to be for cubes, 16 for cones
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
    public JoystickButton GrabberSuckRequested() { //driver/operator will use this method
        return new JoystickButton(armCont, 23); //technically he wanted this button to be for cubes, 24 to intake cones
    }

    @Override
    public JoystickButton GrabberSpitRequested() { //driver/operator will use this method
        return new JoystickButton(armCont, 24); //this should intake cones if he never switches game piece mode
    }

    @Override
    public double GetGrabberPct() { //driver/operator will use this method
        if(armCont.getRawButton(23)) {
            return 5.0; //if backwards switch negative
        } else if (armCont.getRawButton(24)) {
            return -5.0;
        } else {
            return 0.0;
        }
    }

    @Override
    public void initializeButtons(Arm arm, Intake intake, GrabberIntake grabber) {
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
        return new JoystickButton(armCont, XboxController.Button.kRightStick.value);
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
