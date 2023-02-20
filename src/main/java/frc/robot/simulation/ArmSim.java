package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.ArmBrakes;
import frc.robot.interfaces.IArmControl;

public class ArmSim implements IArmControl {
    
    //base robot simulation
    private Mechanism2d m_mech2d;
    private MechanismLigament2d shoulderBar;
    private MechanismLigament2d elbowBar;
    private double elbowDeg;
    private double shoulderDeg;
    private ArmBrakes brakes;

    public ArmSim() {
        brakes = new ArmBrakes();
        shoulderDeg = 60;
        elbowDeg = -60;

        //all units in inches, 0* angle is straight right
        //ligaments are used a lot because they will be shown in the picture, roots are not
        m_mech2d = new Mechanism2d(120, 90);

        //build the field object
        MechanismRoot2d midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0);
        midNodeHome.append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
        MechanismRoot2d highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0);
        highNodeHome.append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
        MechanismRoot2d gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0);
        gridHome.append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));

        //build the robot
        MechanismRoot2d robotBase = m_mech2d.getRoot("Robot Frame", 49.75, 3.5); //edge of grid wall, middle of bumpers now
        robotBase.append(new MechanismLigament2d("Bumpers", 38, 0, 50, new Color8Bit(Color.kDarkGreen)));    //32" robot frame + 3" for bumpers each side
        MechanismRoot2d pivotBase = m_mech2d.getRoot("Pivot Base", 56.25, 16.5);  //edge of grid wall + 3" bumper + 3.5", ~5" frame height + 11.5" height piece
        pivotBase.append(new MechanismLigament2d("Arm Frame", 13, 270, 15, new Color8Bit(Color.kSilver)));
        shoulderBar = new MechanismLigament2d("Shoulder", 36, 0, 15, new Color8Bit(Color.kPurple));
        pivotBase.append(shoulderBar);
        elbowBar = new MechanismLigament2d("Elbow", 28, 0, 15, new Color8Bit(Color.kGold));
        shoulderBar.append(elbowBar);

        // Put Mechanism 2d to SmartDashboard
        SmartDashboard.putData("Arm Sim", m_mech2d);
    }

    public void updateInputs() {
        shoulderBar.setAngle(Rotation2d.fromDegrees(shoulderDeg));
        elbowBar.setAngle(Rotation2d.fromDegrees(elbowDeg - shoulderBar.getAngle()));
    }

    @Override
    public void setShoulderAngle(double angleDeg) {
        shoulderDeg = angleDeg;
        
    }

    @Override
    public void setElbowAngle(double angleDeg) {
        elbowDeg = angleDeg;
        
    }

    @Override
    public void setShoulderMotorVolts(double volts) {
        //System.out.println("Shoulder Volts: " + volts);
        
    }

    @Override
    public void setElbowMotorVolts(double volts) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getElbowAngle() {
        return elbowDeg - shoulderDeg;
    }

    @Override
    public double getShoulderAngle() {
        return shoulderDeg;
    }

    @Override
    public void checkBrake() {
        // TODO Auto-generated method stub
        
    }
}
