// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.livoniawarriors.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.interfaces.IDriveControls;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.interfaces.ITailControl;
import frc.robot.simulation.ArmSim;
import frc.robot.simulation.SwerveDriveSim;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    // robot parts
    private CommandScheduler schedule;

    // robot features
    private ISwerveDrive drive;
    private Odometry odometry;
    private IDriveControls controls;
    private GrabberIntake grabber;
    private Intake intake;
    private Tail tail;

    private PneumaticHub pneumatics;
    private Arm arm;
    public Pose2d startPosition;

    private String[] pdpPracticeChannelNames = {
        "RR Drive",
        "RR Turn",
        "FR Turn",
        "FR Drive",
        "4",
        "5",
        "6",
        "7",
        "Pneumatics",
        "9",
        "10",
        "11",
        "FL Drive",
        "FL Turn",
        "RL Drive",
        "RL Turn"
    };

    private String[] pdhRealChannelNames = {
        null,           //"RF Turn",
        null,           //"RF Drive",
        null,           //"LF Turn",
        null,           //"LF Drive",
        "Elbow",
        "Shoulder",
        null,           //"6",
        null,           //"7",
        "Intake",
        "Front MPM",
        null,           //"10",
        "Tail",
        "Wrist",
        "Back MPM",
        null,           //"14",
        null,           //"15",
        null,           //"RL Turn",
        null,           //"RL Drive",
        null,           //"RR Drive",
        null,           //"RR Turn",
        "Radio Power",
        "Pneumatics",
        "RoboRio",
        null,           //"23",
    };

    private String[] pneumaticNames = {
        null,           //"0",
        null,           //"1",
        "Elbow Brake",
        "Shoulder Brake",
        null,           //"4",
        null,           //"5",
        null,           //"6",
        null,           //"7",
        null,           //"8",
        null,           //"9",
        null,           //"10",
        null,           //"11",
        null,           //"12",
        null,           //"13",
        null,           //"14",
        null,           //"15",
    };

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        pneumatics = new PneumaticHub();
        pneumatics.enableCompressorDigital();

        //internal logger class
        new Logger();
        Logger.RegisterPdp(new PowerDistribution(1,ModuleType.kRev), pdhRealChannelNames);
        Logger.RegisterLoopTimes(this);
        Logger.RegisterPneumaticHub(pneumatics, pneumaticNames);

        // initialize robot parts and locations where they are
        controls = new DriveControls();
       
        // initialize robot features
        schedule = CommandScheduler.getInstance();
        if(isReal()) {
            drive = new SwerveDriveTrain(new SwerveDriveHw());
            arm = new Arm(new ArmHw());
        } else {
            drive = new SwerveDriveTrain(new SwerveDriveSim());
            arm = new Arm(new ArmSim());
        }
        grabber = new GrabberIntake();
        intake = new Intake(new IntakeHw());
        tail = new Tail(new TailHw());

        //subsystems that we don't need to save the reference to, calling new schedules them
        odometry = new Odometry(drive,controls);
        odometry.resetPose(Constants.START_BLUE_LEFT);

        //set the default commands to run
        drive.setDefaultCommand(new DriveStick(drive, controls));
        arm.setDefaultCommand(new DriveArmToPoint(arm, controls));
        tail.setDefaultCommand(new TailMovement(controls, tail));

        controls.ShoulderPosRequested().whileTrue(new ArmManualOverride(arm, controls));
        controls.ShoulderNegRequested().whileTrue(new ArmManualOverride(arm, controls));
        controls.ElbowPosRequested().whileTrue(new ArmManualOverride(arm, controls));
        controls.ElbowNegRequested().whileTrue(new ArmManualOverride(arm, controls));
        controls.ArmToPickupGround().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z));
        controls.ArmToPickupTail().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));
        controls.ArmToPickupHuman().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupHuman_X, Constants.ArmToPickupHuman_Z));
        controls.ArmToSecureLocation().whileTrue(new ArmAutonPoint(arm, Constants.ArmToSecureLocation_X, Constants.ArmToSecureLocation_Z));
        controls.ArmToScoreLow().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreLow_X, Constants.ArmToScoreLow_Z));
        controls.ArmToScoreMiddle().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreMiddle_X, Constants.ArmToScoreMiddle_Z));
        controls.ArmToScoreTop().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z)); //measure these

        controls.GrabberUpRequested().whileTrue(new IntakeMove(controls, intake));
        controls.GrabberDownRequested().whileTrue(new IntakeMove(controls, intake));

        controls.GrabberSuckRequested().whileTrue(new GrabberMove(controls, grabber));
        controls.GrabberSpitRequested().whileTrue(new GrabberMove(controls, grabber));

        SmartDashboard.putData(new MoveWheelsStraight(drive));
        SmartDashboard.putNumber("AutonomousStartPosition", 0);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     */
    @Override
    public void robotPeriodic() {
        //run the command schedule no matter what mode we are in
        schedule.run();
    }
    
    /** This function is called once when autonomous is enabled. */
    @Override
    public void autonomousInit() {
        double AutonomousStartPosition = SmartDashboard.getNumber("AutonomousStartPosition", 0);

        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue){ //Start positions using smartdashboard, red 1-3, blue 1-3
            if(AutonomousStartPosition == 0){
                startPosition = Constants.START_BLUE_LEFT;
            }
            else if(AutonomousStartPosition == 1){
                startPosition = Constants.START_BLUE_MIDDLE;
            }
            else if(AutonomousStartPosition == 2){
                startPosition = Constants.START_BLUE_RIGHT;
            }
            else{
                SmartDashboard.putString("Error", "No Position");
            }
        }
        else if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
            if(AutonomousStartPosition == 0){
                startPosition = Constants.START_RED_LEFT;
            }
            else if(AutonomousStartPosition == 1){
                startPosition = Constants.START_RED_MIDDLE;
            }
            else if(AutonomousStartPosition == 2){
                startPosition = Constants.START_RED_RIGHT;
            }
            else{
                SmartDashboard.putString("Error", "No Position");
            }
        }
        else{
            SmartDashboard.putString("Error", "No Team");
        };

        //set out position to the auto starting position
        odometry.resetPose(startPosition);

        //reset the schedule when auto starts to run the sequence we want
        schedule.cancelAll();

        new SequentialCommandGroup(
            //drive forward 2 sec, turn right, forward 2 sec, left, drive 1 sec
            new DriveTimed(drive, 2),
            new WaitCommand(1.5),
            new DriveTimed(drive, 2)
        );

        //schedule this command for our autonomous
        //schedule.schedule(commands);
       
        //test auto to try driving to spots
        DriveToPoint driveToPoint = new DriveToPoint(drive, odometry, startPosition);
        SmartDashboard.putData(driveToPoint);
        schedule.schedule(driveToPoint);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        //stop all autonomous commands when teleop starts
        //the default commands should take over
        schedule.cancelAll();
        //odometry.resetHeading();
        drive.setDriveMotorBrakeMode(true);
        drive.setTurnMotorBrakeMode(true);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        drive.setDriveMotorBrakeMode(false);
        drive.setTurnMotorBrakeMode(false);
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /* Where to initialize simulation objects */
    @Override
    public void simulationInit() {
    }

    /* where to map simulation physics, like drive commands to encoder counts */
    @Override
    public void simulationPeriodic() {
    }
}
