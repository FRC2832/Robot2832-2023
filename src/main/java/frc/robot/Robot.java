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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.controls.DriveControls;
import frc.robot.controls.LilHaydenDriveControls;
import frc.robot.controls.LilJaydenDriveControls;
import frc.robot.controls.LilJimmyDriveControls;
import frc.robot.controls.LilMickeyDriveControls;
import frc.robot.controls.OperatorControls;
import frc.robot.interfaces.IDriveControls;
import frc.robot.interfaces.IOperatorControls;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.simulation.ArmSim;
import frc.robot.simulation.SwerveDriveSim;
import frc.robot.simulation.TailSim;

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
    private static double batVolt;

    // robot features
    private ISwerveDrive drive;
    public static Odometry odometry;
    private IDriveControls controls;
    private IOperatorControls opControls;
    private GrabberIntake grabber;
    private Intake intake;
    private Tail tail;

    private PneumaticHub pneumatics;
    private Arm arm;
    public Pose2d startPosition;
    private AnalogInput jumper;

    private static boolean pieceMode;

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
    //driver profile variables
    private static final String kDefaultDriver = "Default";
    private static final String kJamesOperator = "James";
    private static final String kHaydenOperator = "Hayden";
    private static final String kMickeyDriver = "Mickey";
    private static final String kJaydenDriver = "Jayden";
    private String driverSelected;
    private String operatorSelected;
    private final SendableChooser<String> driverChooser = new SendableChooser<>();;
    private final SendableChooser<String> operatorChooser = new SendableChooser<>();;
    
    //Autonomus Chooser
    private static final String kNoObstacles = "No Obstacles";
    private static final String kBalence = "Scale";
    private static final String kCord = "Cord";
    private static final String kDoNothing = "Do Nothing";
    private String AutonomousStartPosition;
    private final SendableChooser<String> startPosChooser = new SendableChooser<>();;
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
        opControls = new OperatorControls(); // initialize default operator controls, not used until teleopInit

        //check to see what robot we are
        jumper = new AnalogInput(0);
        var jumperVolts = jumper.getVoltage();

        // initialize robot features
        if (isSimulation() || ((Constants.BuzzVoltage - Constants.JumperError < jumperVolts) && (jumperVolts < Constants.BuzzVoltage + Constants.JumperError))) {
            //either buzz or simulation
            drive = new SwerveDriveTrain(new SwerveDriveSim());
            arm = new Arm(new ArmSim());
            tail = new Tail(new TailSim());
            pdp = new PowerDistribution(0,ModuleType.kCTRE);
            pdpChannelNames = pdpPracticeChannelNames;
            SmartDashboard.putString("Robot", "Simulation/Buzz");
        } else if ((Constants.PracticeVoltage - Constants.JumperError < jumperVolts) && (jumperVolts < Constants.PracticeVoltage + Constants.JumperError)) { 
            //practice chassis
            drive = new SwerveDriveTrain(new SwerveDriveHwPractice());
            arm = new Arm(new ArmSim());
            tail = new Tail(new TailSim());
            pdp = new PowerDistribution(0,ModuleType.kCTRE);
            pdpChannelNames = pdpPracticeChannelNames;
            SmartDashboard.putString("Robot", "Practice");
        } else {
            //real robot
            drive = new SwerveDriveTrain(new SwerveDriveHw());
            arm = new Arm(new ArmHw());
            tail = new Tail(new TailHw());
            pdp = new PowerDistribution(1,ModuleType.kRev);
            pdpChannelNames = pdhRealChannelNames;
            SmartDashboard.putString("Robot", "Real");
        }
        grabber = new GrabberIntake();
        intake = new Intake(new IntakeHw(),arm);
        schedule = CommandScheduler.getInstance();

        //subsystems that we don't need to save the reference to, calling new schedules them
        odometry = new Odometry(drive,controls);
        odometry.resetPose(Constants.START_BLUE_LEFT);

        //set the default commands to run
        drive.setDefaultCommand(new DriveStick(drive, controls));
        arm.setDefaultCommand(new DriveArmToPoint(arm, opControls));
        tail.setDefaultCommand(new TailMovement(controls, tail));
        intake.setDefaultCommand(new IntakeMove(opControls, intake));

        // controls.ShoulderPosRequested().whileTrue(new ArmManualOverride(arm, controls));
        // controls.ShoulderNegRequested().whileTrue(new ArmManualOverride(arm, controls));
        // controls.ElbowPosRequested().whileTrue(new ArmManualOverride(arm, controls));
        // controls.ElbowNegRequested().whileTrue(new ArmManualOverride(arm, controls));
        
        // controls.ArmToPickupGround().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z));
        // controls.ArmToPickupTail().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));
        // controls.ArmToPickupHuman().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupHuman_X, Constants.ArmToPickupHuman_Z));
        // controls.ArmToSecureLocation().whileTrue(new ArmAutonPoint(arm, Constants.ArmToSecureLocation_X, Constants.ArmToSecureLocation_Z));
        // controls.ArmToScoreLow().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreLow_X, Constants.ArmToScoreLow_Z));
        // controls.ArmToScoreMiddle().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreMiddle_X, Constants.ArmToScoreMiddle_Z));
        // controls.ArmToScoreTop().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z)); //measure these
        
        // controls.GrabberUpRequested().whileTrue(new IntakeMove(controls, intake));
        // controls.GrabberDownRequested().whileTrue(new IntakeMove(controls, intake));

        // controls.GrabberSuckRequested().whileTrue(new GrabberMove(controls, grabber));
        // controls.GrabberSpitRequested().whileTrue(new GrabberMove(controls, grabber));

        //controls.ChangePieceMode().toggleOnTrue(new ChangeMode());

        //controls.ChangePieceMode().toggleOnTrue(new ChangeMode()); //whenPressed is deprecated, is there something similar

        SmartDashboard.putData(new MoveWheelsStraight(drive));
        SmartDashboard.putNumber("AutonomousStartPosition", 0);
        SmartDashboard.putString("Error","Ok");
        SmartDashboard.putData(schedule);

        driverChooser.setDefaultOption("Default Settings", kDefaultDriver);        
        driverChooser.addOption("Mickey", kMickeyDriver); 
        driverChooser.addOption("Jayden", kJaydenDriver); 

        operatorChooser.setDefaultOption("Default Settings", kDefaultDriver);      
        operatorChooser.addOption("James", kJamesOperator);
        operatorChooser.addOption("Hayden", kHaydenOperator);
        
        startPosChooser.setDefaultOption("No Obstacles", kNoObstacles);
        startPosChooser.addOption("Scale", kBalence);
        startPosChooser.addOption("Cord", kCord);
        startPosChooser.addOption("Do Nothing", kDoNothing);

        SmartDashboard.putData("Driver Select", driverChooser);
        SmartDashboard.putData("Operator Select", operatorChooser);
        SmartDashboard.putData("StartPos Select", startPosChooser);
        SmartDashboard.putBoolean("Field Oriented", false);
        
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
        var alliance = DriverStation.getAlliance();
        AutonomousStartPosition = startPosChooser.getSelected();

        if (alliance == DriverStation.Alliance.Blue){ //Start positions using smartdashboard, red 1-3, blue 1-3
            if(AutonomousStartPosition.equals(kNoObstacles)){
                startPosition = Constants.START_BLUE_LEFT;
            }
            else if(AutonomousStartPosition.equals(kBalence)){
                startPosition = Constants.START_BLUE_MIDDLE;
            }
            else if(AutonomousStartPosition.equals(kCord)){
                startPosition = Constants.START_BLUE_RIGHT;
            }
            else{
                SmartDashboard.putString("Error", "No Position");
            }
        }
        else if(alliance == DriverStation.Alliance.Red){
            if(AutonomousStartPosition.equals(kNoObstacles)){
                startPosition = Constants.START_RED_LEFT;
            }
            else if(AutonomousStartPosition.equals(kBalence)){
                startPosition = Constants.START_RED_MIDDLE;
            }
            else if(AutonomousStartPosition.equals(kCord)){
                startPosition = Constants.START_RED_RIGHT;
            }
            else{
                SmartDashboard.putString("Error", "No Position");
            }
        }
        else{
            SmartDashboard.putString("Error", "No Team");
        }

        //set out position to the auto starting position
        odometry.resetHeading();
        odometry.resetPose(startPosition);

        //reset the schedule when auto starts to run the sequence we want
        schedule.cancelAll();

        Command sequence;
        if (AutonomousStartPosition.equals(kBalence)) {
            sequence = new SequentialCommandGroup(
                new DriveToScale(drive),
                new DriveToBalance(drive)
            );
        } else if (AutonomousStartPosition.equals(kNoObstacles) || AutonomousStartPosition.equals(kCord)) {
            Pose2d targetPoint;
            double offset;
            if (alliance == DriverStation.Alliance.Red) {
                offset = -4;
            } else {
                offset = 4;
            }
            targetPoint = new Pose2d(startPosition.getX() + offset, startPosition.getY(), startPosition.getRotation());

            sequence = new DriveToPoint(drive,odometry,targetPoint);
        } else {
            sequence = new MoveWheelsStraight(drive);
        }

        //schedule this command for our autonomous
        schedule.schedule(sequence);
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

        //finding which driver or operator is selected
        driverSelected = driverChooser.getSelected();
        operatorSelected = operatorChooser.getSelected();

        if(driverSelected.equals(kMickeyDriver)){
            controls = new LilMickeyDriveControls();
        } else if(driverSelected.equals(kJaydenDriver)){
            controls = new LilJaydenDriveControls();
        } else {}

        if(operatorSelected.equals(kJamesOperator)){
            opControls = new LilJimmyDriveControls();
        } else if(operatorSelected.equals(kHaydenOperator)){
            opControls = new LilHaydenDriveControls();
        } else {}

        //initializes buttons to appropriate mappings
        controls.initializeButtons(arm, intake, grabber);
        opControls.initializeButtons(arm, intake, grabber);
        
        //Reassigning subsystems and default commands with selected driver profiles
        odometry = new Odometry(drive, controls);
        odometry.resetPose(Constants.START_BLUE_LEFT);

        //set the default commands to run
        drive.setDefaultCommand(new DriveStick(drive, controls));
        arm.setDefaultCommand(new DriveArmToPoint(arm, opControls));
        tail.setDefaultCommand(new TailMovement(controls, tail));
        intake.setDefaultCommand(new IntakeMove(opControls, intake));
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double tailDist = tail.getDistSensor();
        if(tailDist > 0 && tailDist < 5.3) {
            tail.setTailAngle(105);
        }
        
        boolean tailUpOverride = SmartDashboard.getBoolean("Distance Sensor Not Working (Override Tail Up)", false);
        if(tailUpOverride) {
            tail.setTailAngle(-8);
        }

        if(arm.getArmXPosition() <= -10) {
            tail.setTailAngle(105);
        }
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


    public static boolean getGamePieceMode(){
        return pieceMode;
    }

    public static void setGamePieceMode(boolean mode){
        pieceMode = mode;
    }

    public static double BatteryVoltage() {
        return batVolt;
    }

}
