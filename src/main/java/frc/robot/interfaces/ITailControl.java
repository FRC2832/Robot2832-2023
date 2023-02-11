package frc.robot.interfaces;


public interface ITailControl {
    void setTailAngle(double angleDeg);


    double getTailAngle();


    void updateInputs();
}
