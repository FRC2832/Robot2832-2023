package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Saitek;
import frc.robot.interfaces.IOperatorControls;
import edu.wpi.first.wpilibj.DriverStation;



public class LilHaydenDriveControls implements IOperatorControls {
    private Saitek armCont;

    public LilHaydenDriveControls() {
        armCont = new Saitek(2); //WARNING: HE USES THE CONTROLLER UPSIDE DOWN; port 2 since Jayden has 2 controllers technically
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
    public JoystickButton ArmToPickupGroundCube(){ //driver/operator will use this method
        return new JoystickButton(armCont, 22); //dummy
    }

    @Override
    public JoystickButton ArmToPickupGroundCone(){ //driver/operator will use this method
        return new JoystickButton(armCont, 18);
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
    public JoystickButton ArmToScoreMiddleFront(){ //driver/operator will use this method
        return new JoystickButton(armCont, 12); 
    }
   
    @Override
    public JoystickButton ArmToScoreTop(){ //driver/operator will use this method
        return new JoystickButton(armCont, 8); //technically he wanted this button to be for cubes, 16 for cones
    }

    @Override
    public JoystickButton ArmToTransitionPoint(){
        return new JoystickButton(armCont, 30);
    }

    @Override
    public JoystickButton IntakeUpRequested() {
        return new JoystickButton(armCont, 23);
    }

    @Override
    public JoystickButton IntakeDownRequested() {
        return new JoystickButton(armCont, 24);
    }

    @Override
    public JoystickButton IntakeSuckRequested() { //driver/operator will use this method
        return new JoystickButton(armCont, 26); //technically he wanted this button to be for cubes, 24 to intake cones
    }

    @Override
    public JoystickButton IntakeSpitRequested() { //driver/operator will use this method
        return new JoystickButton(armCont, 27); //this should intake cones if he never switches game piece mode
    }

    @Override
    public JoystickButton ChangePieceMode() {
        return new JoystickButton(armCont, 25);
    }

    public static boolean checkController() {
        if(DriverStation.getStickAxisCount(2) == 6) {
            if(DriverStation.getStickButtonCount(2) == 26) {
                return true;
            }
        }
        return false;
    }
}
