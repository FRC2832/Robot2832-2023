package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Arm;
import frc.robot.Constants;
import frc.robot.GrabberIntake;
import frc.robot.Intake;
import frc.robot.Saitek;
import frc.robot.commands.ArmAutonPoint;
import frc.robot.commands.ArmManualOverride;
import frc.robot.commands.GrabberMove;
import frc.robot.commands.IntakeMove;
import frc.robot.interfaces.IOperatorControls;

public class OperatorControls implements IOperatorControls {
    private Saitek operCont;

    public OperatorControls() {
        operCont = new Saitek(2);
    }
   
    public double GetArmKinXCommand() {
        return operCont.getxAxis1();
    }

    public double GetArmKinZCommand() {
        return operCont.getyAxis1();
    }

    @Override
    public double GetArmShoulderPct() {
        if(operCont.getOrangeTopLeftButton()) {
            return 0.3;
        } else if (operCont.getOrangeBottomLeftButton()) {
            return -0.3;
        } else {
            return 0;
        }
    }

    @Override
    public double GetArmElbowPct() {
        if(operCont.getOrangeTopRightButton()) {
            return -0.3;
        } else if (operCont.getOrangeBottomRightButton()) {
            return 0.3;
        } else {
            return 0;
        }
    }

    @Override
    public JoystickButton ShoulderPosRequested() {
        return new JoystickButton(operCont, Saitek.Button.orangeTopLeft.value);
    }

    @Override
    public JoystickButton ShoulderNegRequested() {
        return new JoystickButton(operCont, Saitek.Button.orangeBottomLeft.value);
    }

    @Override
    public JoystickButton ElbowPosRequested() {
        return new JoystickButton(operCont, Saitek.Button.orangeTopRight.value);
    }

    @Override
    public JoystickButton ElbowNegRequested() {
        return new JoystickButton(operCont, Saitek.Button.orangeBottomRight.value);
    }

    @Override
    public JoystickButton ArmToPickupGroundCube(){
        return new JoystickButton(operCont, Saitek.Button.yellowTopLeft.value);
    }

    @Override
    public JoystickButton ArmToPickupGroundCone(){
        return new JoystickButton(operCont, Saitek.Button.yellowTopLeft.value);
    }
   
    @Override
    public JoystickButton ArmToPickupTail(){
        return new JoystickButton(operCont, Saitek.Button.yellowBottomLeft.value);
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
        return new JoystickButton(operCont, Saitek.Button.yellowBottomMiddle.value);
    }
   
    @Override
    public JoystickButton ArmToScoreMiddle(){
        return new JoystickButton(operCont, Saitek.Button.yellowBottomRight.value);
    }

    @Override
    public JoystickButton ArmToScoreMiddleFront(){ //driver/operator will use this method
        return new JoystickButton(operCont, Saitek.Button.pinkTopMiddle.value); 
    }
   
    @Override
    public JoystickButton ArmToScoreTop(){
        return new JoystickButton(operCont, Saitek.Button.yellowTopRight.value);
    }

    @Override
    public JoystickButton GrabberUpRequested() {
        return new JoystickButton(operCont, 17);
    }

    @Override
    public JoystickButton GrabberDownRequested() {
        return new JoystickButton(operCont, 19);
    }

    @Override
    public JoystickButton GrabberSuckRequested() {
        return new JoystickButton(operCont, 18);
    }

    @Override
    public JoystickButton GrabberSpitRequested() {
        return new JoystickButton(operCont, 20);
    }

    @Override
    public double GetGrabberPct() {
        if(operCont.getRawButton(17)) {
            return 5.0; //if backwards switch negative
        } else if (operCont.getRawButton(19)) {
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
        ArmToPickupGroundCube().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z));
        ArmToPickupTail().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));
        ArmToPickupHuman().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupHuman_X, Constants.ArmToPickupHuman_Z));
        ArmToSecureLocation().whileTrue(new ArmAutonPoint(arm, Constants.ArmToSecureLocation_X, Constants.ArmToSecureLocation_Z));
        ArmToScoreLow().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreLow_X, Constants.ArmToScoreLow_Z));
        ArmToScoreMiddle().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreMiddle_X, Constants.ArmToScoreMiddle_Z));
        ArmToScoreMiddleFront().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreMiddleFront_X, Constants.ArmToScoreMiddleFront_Z));
        ArmToScoreTop().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z));
        GrabberUpRequested().whileTrue(new IntakeMove(this, intake));
        GrabberDownRequested().whileTrue(new IntakeMove(this, intake));
        GrabberSuckRequested().whileTrue(new GrabberMove(this, grabber));
        GrabberSpitRequested().whileTrue(new GrabberMove(this, grabber));
    }

    @Override
    public JoystickButton ChangePieceMode() {
        return new JoystickButton(operCont, Saitek.Button.pinkTopRight.value);        
    }
}
