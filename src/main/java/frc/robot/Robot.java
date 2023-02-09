// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    // robot parts
    private CommandScheduler schedule;

    // robot features
    private ISwerveDrive drive;
    private Odometry odometry;
    private IDriveControls controls;

    private GrabberIntake intake;

    private PneumaticHub pneumatics;
    private Arm arm;


    private final PowerDistribution pdp = new PowerDistribution(0,ModuleType.kCTRE);
    private NetworkTable table;
    
    // Auton starting position
    public Pose2d startPosition;
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
        new LoopTimeLogger(this);
        pneumatics = new PneumaticHub();
        pneumatics.enableCompressorDigital();

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
        intake = new GrabberIntake();
        
        //subsystems that we don't need to save the reference to, calling new schedules them
        odometry = new Odometry(drive,controls);
        odometry.resetPose(Constants.START_BLUE_LEFT);

        //set the default commands to run
        drive.setDefaultCommand(new DriveStick(drive, controls));
        controls.CubeGrabOpenRequested().whileTrue(new OpenCube(intake));
        controls.CubeGrabCloseRequested().whileTrue(new CloseCube(intake));
        
        arm.setDefaultCommand(new DriveArmToPoint(arm, controls));
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
        loggingPeriodic();
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

      //TODO: Fill in channel names with actual function names
  public String[] pdpChannelNames = {
    "0",
    "1",
    "2",
    "3",
    "4",
    "5",
    "6",
    "7",
    "8",
    "9",
    "10",
    "11",
    "12",
    "13",
    "14",
    "15"
  };

  public void loggingPeriodic() {
    for(int i=0; i<pdpChannelNames.length; i++) {
      table.getEntry("PDP Current " + pdpChannelNames[i]).setDouble(pdp.getCurrent(i));
    }
    table.getEntry("PDP Voltage").setDouble(pdp.getVoltage());
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
}
