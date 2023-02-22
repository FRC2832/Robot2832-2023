// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.interfaces.IDriveControls;
import frc.robot.interfaces.ISwerveDrive;
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
    private final double VOLTS_PER_PSI = 1.931/100; //2.431V at 100psi
    // robot parts
    private CommandScheduler schedule;
    private static double batVolt;

    // robot features
    private ISwerveDrive drive;
    private Odometry odometry;
    private IDriveControls controls;
    private IDriveControls opControls;
    private GrabberIntake grabber;
    private Intake intake;
    private Tail tail;

    private PneumaticHub pneumatics;
    private Arm arm;

    private PowerDistribution pdp;
    private NetworkTable table;

    public Pose2d startPosition;
    private String[] pdpChannelNames;

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
        "RF Turn",
        "RF Drive",
        "LF Turn",
        "LF Drive",
        "Elbow",
        "Shoulder",
        "6",
        "7",
        "Intake",
        "Front MPM",
        "10",
        "Tail",
        "Wrist",
        "Back MPM",
        "14",
        "15",
        "RL Turn",
        "RL Drive",
        "RR Drive",
        "RR Turn",
        "Radio Power",
        "Pneumatics",
        "RoboRio",
        "23",
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
    

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // Starts recording to data log
        DataLogManager.start();
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());
        table = NetworkTableInstance.getDefault().getTable("/status");
        pdp = new PowerDistribution(1,ModuleType.kRev);
        pdpChannelNames = pdhRealChannelNames;
        new LoopTimeLogger(this);
        pneumatics = new PneumaticHub();
        pneumatics.enableCompressorDigital();

        // initialize robot parts and locations where they are
        controls = new DriveControls();
        opControls = new DriveControls(); // initialize default operator controls, not used until teleopInit
       
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
        intake = new Intake(new IntakeHw(), arm);
        tail = new Tail(new TailHw());

        //subsystems that we don't need to save the reference to, calling new schedules them
        odometry = new Odometry(drive,controls);
        odometry.resetPose(Constants.START_BLUE_LEFT);

        //set the default commands to run
        drive.setDefaultCommand(new DriveStick(drive, controls));
        arm.setDefaultCommand(new DriveArmToPoint(arm, controls));
        tail.setDefaultCommand(new TailMovement(controls, tail));
        intake.setDefaultCommand(new IntakeMove(controls, intake));

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

        controls.ChangePieceMode().toggleOnTrue(new ChangeMode()); //whenPressed is deprecated, is there something similar

        SmartDashboard.putData(new MoveWheelsStraight(drive));
        SmartDashboard.putNumber("AutonomousStartPosition", 0);
        SmartDashboard.putData(schedule);

        driverChooser.setDefaultOption("Default Settings", kDefaultDriver);        
        operatorChooser.setDefaultOption("Default Settings", kDefaultDriver);      
        driverChooser.addOption("Mickey", kMickeyDriver); 
        driverChooser.addOption("Jayden", kJaydenDriver); 
        operatorChooser.addOption("James", kJamesOperator);
        operatorChooser.addOption("Hayden", kHaydenOperator);
        SmartDashboard.putData("Driver Select", driverChooser);
        SmartDashboard.putData("Operator Select", operatorChooser);
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
        batVolt = pdp.getVoltage();
        if(count % 2 == 0) {
            loggingPeriodic();
        }
        count++;
    }
    int count = 0;
    
    /** This function is called once when autonomous is enabled. */
    @Override
    public void autonomousInit() {
        double AutonomousStartPosition = SmartDashboard.getNumber("AutonomousStartPosition", 0);

        odometry.resetHeading();

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
        }

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

    public void loggingPeriodic() {
        SmartDashboard.putNumber("Pressure Sensor", (pneumatics.getAnalogVoltage(0) - 0.5) / VOLTS_PER_PSI);
        SmartDashboard.putNumber("Pressure Sensor Voltage", pneumatics.getAnalogVoltage(0));
        for(int i=0; i<pdpChannelNames.length; i++) {
            table.getEntry("PDP Current " + pdpChannelNames[i]).setDouble(pdp.getCurrent(i));
        }
        table.getEntry("PDP Voltage").setDouble(batVolt);
        table.getEntry("PDP Total Current").setDouble(pdp.getTotalCurrent());
        table.getEntry("PDP Temperature").setDouble(pdp.getTemperature());
   
        var canStatus = RobotController.getCANStatus();
        table.getEntry("CAN Bandwidth").setDouble(canStatus.percentBusUtilization);
        table.getEntry("CAN Bus Off Count").setDouble(canStatus.busOffCount);
        table.getEntry("CAN RX Error Count").setDouble(canStatus.receiveErrorCount);
        table.getEntry("CAN Tx Error Count").setDouble(canStatus.transmitErrorCount);
        table.getEntry("CAN Tx Full Count").setDouble(canStatus.txFullCount);

        table.getEntry("Rio 3.3V Voltage").setDouble(RobotController.getVoltage3V3());
        table.getEntry("Rio 5V Voltage").setDouble(RobotController.getVoltage5V());
        table.getEntry("Rio 6V Voltage").setDouble(RobotController.getVoltage6V());
        table.getEntry("Rio 3.3V Current").setDouble(RobotController.getCurrent3V3());
        table.getEntry("Rio 5V Current").setDouble(RobotController.getCurrent5V());
        table.getEntry("Rio 6V Current").setDouble(RobotController.getCurrent6V());
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
