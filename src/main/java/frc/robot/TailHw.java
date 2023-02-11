package frc.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.interfaces.ITailControl;


public class TailHw implements ITailControl{
    TalonSRX tailMotor;
    DutyCycle tailEncoder;
    double tailAngle;


    // TODO: Find following values
    final double MIN_DUTY_CYCLE = 0;
    final double MAX_DUTY_CYCLE = 0;
    final double TAIL_MIN_ANGLE = 0;
    final double TAIL_MAX_ANGLE = 0;
    final double COUNTS_PER_DEGREE_TAIL = 1;


    public TailHw(){
        tailMotor = new TalonSRX(49);
        tailMotor.setNeutralMode(NeutralMode.Brake);
        tailEncoder = new DutyCycle(new DigitalInput(3)); // TODO: Verify channel number
    }


    @Override
    public void setTailAngle(double angleDeg) {
        // TODO Implement Tail PID
        tailMotor.set(ControlMode.Position, angleDeg * COUNTS_PER_DEGREE_TAIL);
    }


    @Override
    public double getTailAngle() {
        return tailAngle;
    }


    @Override
    public void updateInputs() {
        //Frac is 0 at lowest point, 1 at max extension
        var rawDC = tailEncoder.getOutput();
        var dcFrac = (rawDC - MIN_DUTY_CYCLE) / (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);
        tailAngle = TAIL_MIN_ANGLE + dcFrac * (TAIL_MAX_ANGLE - TAIL_MIN_ANGLE);


        tailMotor.setSelectedSensorPosition(tailAngle * COUNTS_PER_DEGREE_TAIL);


    }
}
