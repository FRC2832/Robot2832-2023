package frc.robot;

import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.interfaces.ISwerveDriveIo;

public class SwerveDriveTrain implements ISwerveDrive {
    public static final String MAX_ACCEL_KEY = "Swerve Drive/Max Wheel Accel";
    public static final String MAX_OMEGA_KEY = "Swerve Drive/Max Wheel Omega";

    //these should not be changed as they are "safe" values,
    //use NetworkTables to change them instead in the Preferences table
    public static final double MAX_ACCEL_DEFAULT = 10.0;
    public static final double MAX_OMEGA_DEFAULT = 3000.0;

    private SwerveDriveKinematics kinematics;
    private ISwerveDriveIo hardware;
    private SwerveModulePosition[] swerveStates;
    private SwerveModuleState[] swerveTargets;
    private Pose2d robotPose;
    private static String moduleNames[];
    private double swerveOffsets[];
    private double turnOffsets[];
    private InterpolatingTreeMap<Double, Double> speedReduction;
    private double gyroOffset = 0;
    private PIDController pidZero = new PIDController(0.15, 0.001, 0);
    private PIDController[] drivePid;
    private PIDController[] turnPid;
    private SwerveModuleState[] currentState;

    TrapezoidProfile.Constraints constraints;
    TrapezoidProfile.State[] lastState;

    // Initializer block starts
    static {
        //initialize module names
        moduleNames = new String[Constants.NUM_WHEELS];
        moduleNames[FL] = "Swerve FL ";
        moduleNames[FR] = "Swerve FR ";
        moduleNames[RL] = "Swerve RL ";
        moduleNames[RR] = "Swerve RR ";
    }

    public SwerveDriveTrain(ISwerveDriveIo hSwerveDriveIo) {
        register();
        this.hardware = hSwerveDriveIo;

        //initialize the corner locations
        kinematics = new SwerveDriveKinematics(hSwerveDriveIo.getCornerLocations());
        hardware.setKinematics(kinematics);
        
        //initialize the swerve states
        swerveStates = new SwerveModulePosition[Constants.NUM_WHEELS];
        swerveTargets = new SwerveModuleState[Constants.NUM_WHEELS];
        drivePid = new PIDController[Constants.NUM_WHEELS];
        turnPid = new PIDController[Constants.NUM_WHEELS];
        for(int wheel = 0; wheel < Constants.NUM_WHEELS; wheel++) {
            swerveStates[wheel] = new SwerveModulePosition();
            swerveTargets[wheel] = new SwerveModuleState();
            drivePid[wheel] = new PIDController(0.5, 0.03, 0.0);
            turnPid[wheel] = new PIDController(5,1.8,0);
        }

        //initialize the swerve offsets
        swerveOffsets = new double[Constants.NUM_WHEELS];
        swerveOffsets[FL] = hardware.getWheelOffset(FL);
        swerveOffsets[FR] = hardware.getWheelOffset(FR);
        swerveOffsets[RL] = hardware.getWheelOffset(RL);
        swerveOffsets[RR] = hardware.getWheelOffset(RR);

        hardware.updateInputs();
        turnOffsets = new double[Constants.NUM_WHEELS];
        for(int i=0; i<turnOffsets.length; i++) {
            double offset = (swerveOffsets[i] - hardware.getCornerAbsAngle(i));
            if (offset > 180) {
                offset -= 360;
            } else if (offset < -180) {
                offset += 360;
            }
            turnOffsets[i] = offset + hardware.getCornerAngle(i);
        }
        gyroOffset = getHeading().getDegrees();

        //input is angle off desired, output is percent reduction
        speedReduction = new InterpolatingTreeMap<Double, Double>();
        speedReduction.put(0., 1.);
        speedReduction.put(45., 1.);
        speedReduction.put(90., 1.);

        //setup for motion profiling
        constraints = new TrapezoidProfile.Constraints(Constants.MAX_DRIVETRAIN_SPEED, Constants.MAX_DRIVETRAIN_SPEED);
        lastState = new TrapezoidProfile.State[Constants.NUM_WHEELS];
        for(int i=0; i<Constants.NUM_WHEELS; i++) {
            lastState[i] = new TrapezoidProfile.State(0, 0.0);
        }

        //write the preference keys if they aren't there
        if(!Preferences.containsKey(MAX_ACCEL_KEY)) {
            Preferences.setDouble(MAX_ACCEL_KEY, MAX_ACCEL_DEFAULT);
        }
        if(!Preferences.containsKey(MAX_OMEGA_KEY)) {
            Preferences.setDouble(MAX_OMEGA_KEY, MAX_OMEGA_DEFAULT);
        }
    }
    
    @Override
    public void periodic() {
        hardware.updateInputs();
        currentState = new SwerveModuleState[Constants.NUM_WHEELS];

        //read the swerve corner state
        for(int wheel = 0; wheel < Constants.NUM_WHEELS; wheel++) {
            swerveStates[wheel].distanceMeters = hardware.getCornerDistance(wheel);

            double angle = hardware.getCornerAbsAngle(wheel) - swerveOffsets[wheel];
            swerveStates[wheel].angle = Rotation2d.fromDegrees(angle);

            currentState[wheel] = new SwerveModuleState();
            currentState[wheel].angle = swerveStates[wheel].angle;
            currentState[wheel].speedMetersPerSecond = hardware.getCornerSpeed(wheel);

            if(DriverStation.isDisabled()) {
                turnPid[wheel].reset();
            }
        }

        Logger.PushSwerveStates(currentState,swerveTargets);
        //display data on SmartDashboard
        for(int wheel=0; wheel < Constants.NUM_WHEELS; wheel++) {
            SmartDashboard.putNumber(moduleNames[wheel] + "Calc Angle", swerveStates[wheel].angle.getDegrees());
        }
    }

    @Override
    public void SwerveDrive(double xSpeed, double ySpeed, double turn, boolean fieldOriented) {
        // ask the kinematics to determine our swerve command
        ChassisSpeeds speeds;

        double currentHeading = getHeading().getDegrees();
        if (Math.abs(turn) > 0.1) {
            //if a turn is requested, reset the zero for the drivetrain
            gyroOffset = currentHeading;
            pidZero.reset();
        } else {
            //straighten the robot
            turn = pidZero.calculate(currentHeading,gyroOffset);
        }

        if (fieldOriented) {
            var angle = robotPose.getRotation();
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turn, angle);
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, turn);
        }
        
        SmartDashboard.putNumber("Swerve XSpeed", xSpeed);
        SmartDashboard.putNumber("Swerve YSpeed", ySpeed);
        SmartDashboard.putNumber("Swerve Turn", turn);

        //calculate the states from the speeds
        SwerveModuleState[] requestStates = kinematics.toSwerveModuleStates(speeds);
        // sometime the Kinematics spits out too fast of speeds, so this will fix this
        SwerveDriveKinematics.desaturateWheelSpeeds(requestStates, Constants.MAX_DRIVETRAIN_SPEED);

        //filter the swerve wheels
        requestStates = optomizeSwerve(requestStates, currentState);

        // command each swerve module
        for (int i = 0; i < requestStates.length; i++) {
            //turn PID
            if (Math.abs(swerveStates[i].angle.minus(requestStates[i].angle).getDegrees()) < 1) {
                turnPid[i].reset();
            }
            var turnVolts = -turnPid[i].calculate(swerveStates[i].angle.getRadians(), requestStates[i].angle.getRadians());
            hardware.setTurnCommand(i, ControlMode.PercentOutput, turnVolts / RobotController.getBatteryVoltage());
            
            // voltage drive mode
            //var driveVolts = drivePid[i].calculate(hardware.getCornerSpeed(i),requestStates[i].speedMetersPerSecond);
            //var ff = requestStates[i].speedMetersPerSecond / Constants.MAX_DRIVETRAIN_SPEED * RobotController.getBatteryVoltage();
            //driveVolts += ff;
            //hardware.setDriveCommand(i, ControlMode.PercentOutput, driveVolts / RobotController.getBatteryVoltage());

            // velocity drive mode
            hardware.setDriveCommand(i, ControlMode.Velocity, requestStates[i].speedMetersPerSecond);

            //motion magic mode (note, motion magic only works on position PID control, so we will make "fake" distance points to go to)
            //TrapezoidProfile.State newState = new TrapezoidProfile.State(lastState[i].position, requestStates[i].speedMetersPerSecond);
            //TrapezoidProfile profile = new TrapezoidProfile(constraints, newState, lastState[i]);
            //lastState[i] = profile.calculate(Constants.LOOP_TIME);
            //hardware.setDriveCommand(i, ControlMode.MotionMagic, lastState[i].position);

            SmartDashboard.putNumber(moduleNames[i] + "Command Angle", requestStates[i].angle.getDegrees());
            SmartDashboard.putNumber(moduleNames[i] + "Command Speed", requestStates[i].speedMetersPerSecond);
        }
        swerveTargets = requestStates;
    }

    public void setWheelCommand(SwerveModuleState[] requestStates) {
        swerveTargets = requestStates;
        for(int i=0; i<requestStates.length; i++) {
            requestStates[i] = SwerveModuleState.optimize(requestStates[i], swerveStates[i].angle);
                
            var volts = -turnPid[i].calculate(swerveStates[i].angle.getRadians(),requestStates[i].angle.getRadians());
            hardware.setDriveCommand(i, ControlMode.PercentOutput, requestStates[i].speedMetersPerSecond / Constants.MAX_DRIVETRAIN_SPEED);
            hardware.setTurnCommand(i, ControlMode.PercentOutput, volts / RobotController.getBatteryVoltage());
        }
    }

    public static SwerveModuleState[] optomizeSwerve(SwerveModuleState[] requestStates, SwerveModuleState[] currentState) {
        SwerveModuleState[] outputStates = new SwerveModuleState[requestStates.length];
        // command each swerve module
        for (int i = 0; i < requestStates.length; i++) {
            SmartDashboard.putNumber(moduleNames[i] + "Requested Angle", requestStates[i].angle.getDegrees());
            SmartDashboard.putNumber(moduleNames[i] + "Requested Speed", requestStates[i].speedMetersPerSecond);
            outputStates[i] = new SwerveModuleState();

            //smooth out drive command
            double maxAccel = Preferences.getDouble(MAX_ACCEL_KEY, MAX_ACCEL_DEFAULT);
            double maxSpeedDelta = maxAccel * Constants.LOOP_TIME;           //acceleration * loop time
            //whatever value is bigger flips when forwards vs backwards
            double value1 = currentState[i].speedMetersPerSecond - maxSpeedDelta;
            double value2 = currentState[i].speedMetersPerSecond + maxSpeedDelta;
            outputStates[i].speedMetersPerSecond = MathUtil.clamp(
                requestStates[i].speedMetersPerSecond,                  //current request
                Math.min(value1, value2),                               //last request minimum
                Math.max(value1, value2));                              //last request maximum


            //smooth out turn command
            double maxOmega = Preferences.getDouble(MAX_OMEGA_KEY, MAX_OMEGA_DEFAULT);
            double maxAngleDelta = maxOmega * Constants.LOOP_TIME;           //acceleration * loop time
            double curAngle = currentState[i].angle.getDegrees();

            double angleReq = requestStates[i].angle.getDegrees();
            if (Math.abs(requestStates[i].speedMetersPerSecond) > maxSpeedDelta) {
                angleReq = MathUtil.inputModulus(angleReq, curAngle - 180, curAngle + 180);
            } else {
                angleReq = curAngle;
            }

            outputStates[i].angle = Rotation2d.fromDegrees(MathUtil.clamp(
                angleReq,                   //current request
                curAngle - maxAngleDelta,        //last request minimum
                curAngle + maxAngleDelta));      //last request maximum

            //check to see if the robot request is moving
            if (Math.abs(requestStates[i].speedMetersPerSecond) < maxSpeedDelta) {
                //stop the requests if there is no movement
                outputStates[i].angle = currentState[i].angle;
                //take out minimal speed so that the motors don't jitter
                outputStates[i].speedMetersPerSecond = 0;
            }
        }
        return outputStates;
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public Rotation2d getHeading() {
        return hardware.getHeading();
    }

    @Override
    public double getPitch() {
        return hardware.getPitch();
    }

    @Override
    public SwerveModulePosition[] getSwerveStates() {
        return swerveStates;
    }

    @Override
    public void setPose(Pose2d robotPose) {
        this.robotPose = robotPose;        
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        hardware.setTurnMotorBrakeMode(brakeOn);
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        hardware.setDriveMotorBrakeMode(brakeOn);
    }

    @Override
    public Translation2d[] getCornerLocations() {
        return hardware.getCornerLocations();
    }
}
