// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.livoniawarriors.Logger;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    public static final boolean CUBE_MODE = true;
    public static final boolean CONE_MODE = false;

    // robot parts
    private CommandScheduler schedule;

    // robot features
    private ISwerveDrive drive;
    public static Odometry odometry;
    private IDriveControls controls;
    private IOperatorControls opControls;
    private Intake intake;
    private Pivot pivot;
    private Tail tail;
    public static String SerialNumber;
    private PneumaticHub pneumatics;
    private Arm arm;
    private LED_controller leds;

    private static boolean pieceMode = CUBE_MODE;

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
        "RF Turn", 
        "RF Drive",
        "LF Turn", 
        "LF Drive",
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
        "Pneumatics",
        "RL Turn", 
        "RL Drive",
        "RR Drive",
        "RR Turn", 
        "Radio Power",
        null,
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
    private final SendableChooser<String> driverChooser = new SendableChooser<>();
    private final SendableChooser<String> operatorChooser = new SendableChooser<>();
    public Logger logger = new Logger();
    
    //Autonomus Chooser
    private AutonChooser auton;
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        //internal logger class
        SmartDashboard.putString("Serial Number", RobotController.getSerialNumber());
        SerialNumber = RobotController.getSerialNumber();
        Logger.RegisterLoopTimes(this);
        RobotController.setBrownoutVoltage(3);

        // initialize robot parts and locations where they are
        controls = new DriveControls();
        opControls = new OperatorControls(); // initialize default operator controls, not used until teleopInit

        // initialize robot features
        if (isSimulation() || (SerialNumber.equals("031b525b"))) {
            //either buzz or simulation
            drive = new SwerveDriveTrain(new SwerveDriveSim());
            arm = new Arm(new ArmSim());
            tail = new Tail(new TailSim());
            Logger.RegisterPdp(new PowerDistribution(0,ModuleType.kCTRE), pdpPracticeChannelNames);
            SmartDashboard.putString("Robot", "Simulation/Buzz");
        } else if (SerialNumber.equals("031e3219")) { 
            //practice chassis
            SmartDashboard.putString("Robot", "Practice");
            drive = new SwerveDriveTrain(new SwerveDriveHwPractice());
            arm = new Arm(new ArmSim());
            tail = new Tail(new TailSim());
            Logger.RegisterPdp(new PowerDistribution(0,ModuleType.kCTRE), pdpPracticeChannelNames);
            
        } else {
            //real robot
            SmartDashboard.putString("Robot", "Real");
            pneumatics = new PneumaticHub();
            pneumatics.enableCompressorDigital();
            Logger.RegisterPneumaticHub(pneumatics, pneumaticNames);

            drive = new SwerveDriveTrain(new SwerveDriveHw());
            arm = new Arm(new ArmHw());
            tail = new Tail(new TailHw());
            Logger.RegisterPdp(new PowerDistribution(1,ModuleType.kRev), pdhRealChannelNames);
            
        }
        intake = new Intake();
        pivot = new Pivot(new PivotHw(),arm);
        schedule = CommandScheduler.getInstance();
        leds = new LED_controller();

        //subsystems that we don't need to save the reference to, calling new schedules them
        odometry = new Odometry(drive,controls, arm, tail);
        odometry.resetPose(Constants.START_BLUE_LEFT);

        SmartDashboard.putData(new ChangeMode());

        SmartDashboard.putData(new MoveWheelsStraight(drive));
        SmartDashboard.putNumber("AutonomousStartPosition", 0);
        SmartDashboard.putString("Error","Ok");
        SmartDashboard.putData(schedule);

        driverChooser.addOption("Default Settings", kDefaultDriver);        
        driverChooser.setDefaultOption("Mickey", kMickeyDriver); 
        driverChooser.addOption("Jayden", kJaydenDriver); 

        operatorChooser.addOption("Default Settings", kDefaultDriver);      
        operatorChooser.setDefaultOption("James", kJamesOperator);
        operatorChooser.addOption("Hayden", kHaydenOperator);

        SmartDashboard.putData("Driver Select", driverChooser);
        SmartDashboard.putData("Operator Select", operatorChooser);
        SmartDashboard.putBoolean("Field Oriented", false);

        //Construct auton thing
        auton = new AutonChooser(drive, odometry, intake, arm);
        
        logger.start();
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
        leds.update(drive, opControls);
    }
    
    /** This function is called once when autonomous is enabled. */
    @Override
    public void autonomousInit() {
        drive.setDriveMotorBrakeMode(true);
        drive.setTurnMotorBrakeMode(true);
        
        //force the sticky faults to clear at Autonomous
        SmartDashboard.putBoolean("Clear Faults", true);

        //set out position to the auto starting position
        odometry.resetHeading();
        auton.setStartPos();
        odometry.resetPose(auton.getStartPos());

        //reset the schedule when auto starts to run the sequence we want
        schedule.cancelAll();

        //schedule the command for our autonomous
        schedule.schedule(auton.getAuton());
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
        } else {
            controls = new DriveControls();
            
        }

        if(operatorSelected.equals(kJamesOperator)){
            opControls = new LilJimmyDriveControls();
        } else if(operatorSelected.equals(kHaydenOperator)){
            opControls = new LilHaydenDriveControls();
        } else {
            opControls = new OperatorControls();
        }

        //set the default commands to run
        drive.setDefaultCommand(new DriveStick(drive, controls));
        arm.setDefaultCommand(new DriveArmToPoint(arm, opControls));
        tail.setDefaultCommand(new TailMovement(controls, tail, arm));
        pivot.setDefaultCommand(new PivotMove(opControls, pivot));
        intake.setDefaultCommand(new IntakeMove(opControls, intake));
        odometry.setDriverControls(controls);

        //set all the other commands
        opControls.ShoulderPosRequested().whileTrue(new ArmManualOverride(arm, opControls));
        opControls.ShoulderNegRequested().whileTrue(new ArmManualOverride(arm, opControls));
        opControls.ElbowPosRequested().whileTrue(new ArmManualOverride(arm, opControls));
        opControls.ElbowNegRequested().whileTrue(new ArmManualOverride(arm, opControls));
        opControls.ArmToPickupGroundCone().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z));
        opControls.ArmToPickupGroundCube().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z));
        opControls.ArmToPickupTail().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));
        opControls.ArmToPickupHuman().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupHuman_X, Constants.ArmToPickupHuman_Z));
        opControls.ArmToSecureLocation().whileTrue(new ArmAutonPoint(arm, Constants.ArmToSecureLocation_X, Constants.ArmToSecureLocation_Z));
        opControls.ArmToScoreLow().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreLow_X, Constants.ArmToScoreLow_Z));
        opControls.ArmToScoreMiddle().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreMiddle_X, Constants.ArmToScoreMiddle_Z));
        opControls.ArmToScoreMiddleFront().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreMiddleFront_X, Constants.ArmToScoreMiddleFront_Z));
        opControls.ArmToScoreTop().whileTrue(new ArmAutonPoint(arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z));
        opControls.ArmToPickupHuman().whileTrue(new ArmAutonPoint(arm, Constants.ArmToPickupHuman_X, Constants.ArmToPickupHuman_Z).alongWith(new PivotSetPoint(pivot, Constants.PivotToPickupHuman)));
        opControls.IntakeSuckRequested().whileTrue(new IntakeMove(opControls, intake));
        opControls.IntakeSpitRequested().whileTrue(new IntakeMove(opControls, intake));
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
        //check if the controllers are connected well
        driverSelected = driverChooser.getSelected();
        operatorSelected = operatorChooser.getSelected();

        boolean valid;
        if(driverSelected.equals(kMickeyDriver)){
            valid = LilMickeyDriveControls.checkController();
        } else if(driverSelected.equals(kJaydenDriver)){
            valid = LilJaydenDriveControls.checkController();
        } else {
            valid = DriveControls.checkController();
        }
        SmartDashboard.putBoolean("Driver Check", valid);

        if(operatorSelected.equals(kJamesOperator)){
            valid = LilJimmyDriveControls.checkController();
        } else if(operatorSelected.equals(kHaydenOperator)){
            valid = LilHaydenDriveControls.checkController();
        } else {
            valid = OperatorControls.checkController();
        }
        SmartDashboard.putBoolean("Operator Check", valid);   
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
}
