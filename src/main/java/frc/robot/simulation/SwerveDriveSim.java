package frc.robot.simulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.interfaces.ISwerveDriveIo;

public class SwerveDriveSim implements ISwerveDriveIo {
    private SwerveModuleState swerveStates[];
    private SwerveDriveKinematics kinematics;
    private double chassisAngle;

    private double absAngle[];
    private double turnAngle[];
    private double driveSpeed[];

    private FlywheelSim turnMotorSim[];
    private PIDController turningPIDController[];

    public SwerveDriveSim() {
        swerveStates = new SwerveModuleState[Constants.NUM_WHEELS];
        absAngle = new double[Constants.NUM_WHEELS];
        turnAngle = new double[Constants.NUM_WHEELS];
        driveSpeed = new double[Constants.NUM_WHEELS];
        for(int i=0; i<swerveStates.length; i++) {
            swerveStates[i] = new SwerveModuleState();
        }
        chassisAngle = 0;

        turnMotorSim = new FlywheelSim[Constants.NUM_WHEELS];
        turningPIDController = new PIDController[Constants.NUM_WHEELS];
        for(int i=0; i<Constants.NUM_WHEELS; i++) {
            turnMotorSim[i] = new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.6095, 0.0648),
                DCMotor.getFalcon500(1), 150f/7);
            turningPIDController[i] = new PIDController(5.0, 0.0, 50.0, 0.001);
        }
    }

    @Override
    public void setKinematics(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }
    
    @Override
    public void updateInputs() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(swerveStates);
        
        chassisAngle += Math.toDegrees(speeds.omegaRadiansPerSecond * Constants.LOOP_TIME);
        
        //xyz_mps[0] = speeds.vxMetersPerSecond / Constants.LOOP_TIME;
        //xyz_mps[1] = speeds.vyMetersPerSecond / Constants.LOOP_TIME;

        //TODO: Simulate the actual swerve corners... https://www.chiefdelphi.com/t/sysid-gains-on-sds-mk4i-modules/400373/7
        for(int i=0; i<swerveStates.length; i++) {
            driveSpeed[i] = swerveStates[i].speedMetersPerSecond;
            //turnAngle[i] = swerveStates[i].angle.getDegrees();
            //absAngle[i] = turnAngle[i];

            //run at 1ms rate to simulate hardware
            for(var loops =0; loops < Constants.LOOP_TIME / 0.001; loops++) {
                //use the PID to calculate a voltage to apply to the motor
                double turnOutput = turningPIDController[i].calculate(Math.toRadians(absAngle[i]), swerveStates[i].angle.getRadians());
                //check if we exceed battery voltage
                if(turnOutput > Constants.NOM_BATTERY_VOLTAGE) {
                    turnOutput = Constants.NOM_BATTERY_VOLTAGE;
                } else if (turnOutput < -Constants.NOM_BATTERY_VOLTAGE) {
                    turnOutput = -Constants.NOM_BATTERY_VOLTAGE;
                } else {
                    //leave voltage as is
                }
                //set the input
                turnMotorSim[i].setInputVoltage(turnOutput);
                //run it
                turnMotorSim[i].update(0.001);
                //figure out how much angle changed per loop
                double angChange = Math.toDegrees(turnMotorSim[i].getAngularVelocityRadPerSec() * 0.001);
                //update the sensor values
                turnAngle[i] += angChange;
                absAngle[i] += angChange;
            }
        }
        
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(chassisAngle);
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        // Not a Sim function
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        // Not a Sim function
    }

    @Override
    public double getCornerAbsAngle(int wheel) {
        return absAngle[wheel];
    }

    @Override
    public double getCornerAngle(int wheel) {
        return turnAngle[wheel];
    }

    @Override
    public double getCornerSpeed(int wheel) {
        return driveSpeed[wheel];
    }

    @Override
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        swerveStates[wheel] = swerveModuleState;
    }
    
}
