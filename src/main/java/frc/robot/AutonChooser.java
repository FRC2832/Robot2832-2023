package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import org.livoniawarriors.UtilFunctions;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmAutonPoint;
import frc.robot.commands.ArmAutonPointNoLimit;
import frc.robot.commands.DriveToBalance;
import frc.robot.commands.DriveToOffScale;
import frc.robot.commands.DriveToOffScaleNegative;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToScale;
import frc.robot.commands.DriveToScaleNegative;
import frc.robot.commands.IntakeBackward;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.MoveWheelsSideways;
import frc.robot.commands.MoveWheelsStraight;
import frc.robot.commands.PivotSetPoint;
import frc.robot.interfaces.ISwerveDrive;

public class AutonChooser {
    private Command sequence;
    private ISwerveDrive drive;
    private Intake intake;
    private Arm arm;
    private Pivot pivot;
    private Odometry odometry;
    public Pose2d startPosition;
    private final SendableChooser<String> startPosChooser = new SendableChooser<>();
    private Alliance alliance;

    //Autonomous Chooser
    private static final String kNoObstacles = "No Obstacles";
    private static final String kBalance = "Scale";
    private static final String kMobility = "Mobility";
    private static final String kMobilityBack = "Scale Back";
    private static final String kCord = "Cord";
    //private static final String kDoNothing = "Do Nothing";
    private static final String kOnePiece = "spit";
    private static final String kTwoPiece = "Score Two";
    private static final String kTwoPieceCord = "Score Two Cord";
    private static final String kThreePiece = "Score Three";
    private static final String kPathPlan = "Drive Around Balance";
    private static final String kPlan3P = "3 piece pathplan";
    private static final String kThanks = "Thanksgiving Special";

    private String AutonomousStartPosition;

    public AutonChooser(ISwerveDrive drive, Odometry odometry, Intake intake, Arm arm, Pivot pivot){
        this.drive = drive;
        this.odometry = odometry;
        this.intake = intake;
        this.arm = arm;
        this.pivot = pivot;

        sequence = new DriveToPoint(drive, odometry, odometry.getPose()); //if no sequence gets loaded it'll go to where it is (do nothing)

        startPosChooser.addOption("Scale", kBalance);
        startPosChooser.setDefaultOption("Scale + Mobility", kMobility);
        startPosChooser.addOption("Scale + Mobility Backwards", kMobilityBack);
        startPosChooser.addOption("No Obstacles", kNoObstacles);
        startPosChooser.addOption("Cord", kCord);
        //startPosChooser.addOption("Do Nothing", kDoNothing);
        startPosChooser.addOption("Score One", kOnePiece);
        startPosChooser.addOption("Score Two Clear", kTwoPiece);
        startPosChooser.addOption("Score Two Cord", kTwoPieceCord);
        startPosChooser.addOption("Score Three", kThreePiece);
        //startPosChooser.addOption(kPathPlan, kPathPlan);
        startPosChooser.addOption(kPlan3P, kPlan3P);
        startPosChooser.addOption(kThanks, kThanks);
        
        SmartDashboard.putData("StartPos Select", startPosChooser);

        setStartPos();
    }

    public Command getAuton(){
        if (AutonomousStartPosition.equals(kBalance)) {
            sequence = autoBalance();
        } else if (AutonomousStartPosition.equals(kMobility)) {
            sequence = autoBalanceAndOut();
        } else if (AutonomousStartPosition.equals(kMobilityBack)) {
            sequence = autoBalanceAndOutBackwards();
        } else if (AutonomousStartPosition.equals(kNoObstacles)) {
            sequence = autoNoObstacles();
        } else if (AutonomousStartPosition.equals(kCord)) {
            sequence = autoCord();
        } else if (AutonomousStartPosition.equals(kOnePiece)){ //score on top row
            sequence = autoSpit();
        } else if (AutonomousStartPosition.equals(kTwoPiece)){ //score two somewhere
            sequence = autoTwoPiece(true);
        } else if (AutonomousStartPosition.equals(kTwoPieceCord)){ //score two somewhere
            sequence = autoTwoPiece(false);
        } else if (AutonomousStartPosition.equals(kThreePiece)){ //score three somewhere
            sequence = autoThreePiece();
        } else if (AutonomousStartPosition.equals(kPathPlan)){ //path plan
            sequence = autoPathPlanner();
        } else if (AutonomousStartPosition.equals(kPlan3P)){ //path plan 3 piece
            sequence = autoPlanThreePiece();
        } else if (AutonomousStartPosition.equals(kThanks)){ //path plan 3 piece
            sequence = autoThanksgiving();
        } else {
            sequence = new MoveWheelsStraight(drive);
        }
        return sequence;
    }

    public Pose2d getStartPos(){
        return startPosition;
    }

    //ALL AUTON SEQUENCES
    public Command autoBalance(){ //balance auto from jackson comp
        Command tempSequence;
        tempSequence = new ArmAutonPointNoLimit(this.arm, Constants.ArmToScoreTopAuto_X, Constants.ArmToScoreTopAuto_Z)
                .deadlineWith(new MoveWheelsStraight(drive));
        
        tempSequence = tempSequence.andThen(spit());
    
        tempSequence = tempSequence.andThen(new WaitCommand(0.5)
            .andThen(new DriveToScale(drive))
            .andThen(new DriveToBalance(drive))
            .alongWith(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z)));
        return tempSequence;
    }    

    public Command autoBalanceAndOut(){ //drive over charge station out of community then come back in and balance - untested
        Command tempSequence;
        tempSequence = new ArmAutonPointNoLimit(this.arm, Constants.ArmToScoreTopAuto_X, Constants.ArmToScoreTopAuto_Z)
            .deadlineWith(new MoveWheelsStraight(drive));   
        tempSequence = tempSequence.andThen(spit());

        // tempSequence = tempSequence.andThen(new WaitCommand(0.5)
        //     .andThen(new DriveToScale(drive)).withTimeout(2.9)
        //     .deadlineWith(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z))
        //     .andThen(new DriveToOffScale(drive))
        //     .andThen(new DriveToScaleNegative(drive))
        //     .andThen(new DriveToBalance(drive)));
        tempSequence = tempSequence.andThen(new WaitCommand(0.5)
            .andThen(Commands.parallel(new DriveToScale(drive),
            new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z))).withTimeout(2.9)
            .andThen(new DriveToOffScale(drive))
            .andThen(new DriveToScaleNegative(drive))
            .andThen(new DriveToBalance(drive)));
        return tempSequence;
    }

    public Command autoBalanceAndOutBackwards(){ //drive over charge station out of community then come back in and balance - untested
        //add something to re field orient the right way
        Command tempSequence;
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreMiddleFront_X, Constants.ArmToScoreMiddleFront_Z)
            .deadlineWith(new MoveWheelsStraight(drive));   
        tempSequence = tempSequence.andThen(spit());
        //tempSequence = spit();

        tempSequence = tempSequence.andThen(new WaitCommand(0.5)
            .andThen(new DriveToScaleNegative(drive)).withTimeout(2.9)
            .deadlineWith(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z))
            .andThen(new DriveToOffScaleNegative(drive))
            .andThen(new DriveToScale(drive))
            .andThen(new DriveToBalance(drive)));
        return tempSequence;
    }

    public Command autoNoObstacles(){
        Command tempSequence;
        Pose2d targetPoint;
        
        targetPoint = new Pose2d(startPosition.getX() + offsetX(4), startPosition.getY(), startPosition.getRotation());
            
        tempSequence = new ArmAutonPointNoLimit(this.arm, Constants.ArmToScoreTopAuto_X, Constants.ArmToScoreTopAuto_Z)
            .deadlineWith(new MoveWheelsStraight(drive));
            
        tempSequence = tempSequence.andThen(spit());

        tempSequence = tempSequence.andThen(new WaitCommand(0.5)
            .andThen(Commands.parallel((new DriveToPoint(drive,odometry,targetPoint)),
            (new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z)))));
        return tempSequence;
    }

    public Command autoCord(){
        Command tempSequence;
        Pose2d targetPoint;
        
        targetPoint = new Pose2d(startPosition.getX() + offsetX(5), startPosition.getY(), startPosition.getRotation());
            
        tempSequence = new ArmAutonPointNoLimit(this.arm, Constants.ArmToScoreTopAuto_X, Constants.ArmToScoreTopAuto_Z)
            .deadlineWith(new MoveWheelsStraight(drive));
        
        tempSequence = tempSequence.andThen(spit());
        
        tempSequence = tempSequence.andThen(new WaitCommand(0.5)
            .andThen(new DriveToPoint(drive,odometry,targetPoint))
            .alongWith(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z)));
        return tempSequence;
    }

    /*
    public Command autoSpit(){
        Command tempSequence;
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z);
        tempSequence = tempSequence.andThen(spit());
        tempSequence = tempSequence.andThen(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));
        return tempSequence;
    }
    */

    public Command autoSpit(){
        Command tempSequence = spit();
        return tempSequence;
    }

    public Command autoTwoPiece(boolean isClear){
        Command tempSequence;
        Pose2d targetPoint;
        double intakeAngle = UtilFunctions.getSetting("Auto2P/Intake Angle", 90);
        //Put arm in scoring position
        tempSequence = new ArmAutonPointNoLimit(this.arm, Constants.ArmToScoreTopAuto_X, Constants.ArmToScoreTopAuto_Z)
            .deadlineWith(new MoveWheelsStraight(drive));
        
        //score 1st piece
        tempSequence = tempSequence.andThen(spit());
        
        if(isClear){
            //drive to next piece and lower arm
            targetPoint = new Pose2d(startPosition.getX() + offsetX(4.6), startPosition.getY(), startPosition.getRotation());
            //pickup 2nd
            tempSequence = tempSequence//.andThen(new WaitCommand(0.5))
                .andThen(Commands.parallel(
                    new DriveToPoint(drive,odometry,targetPoint,1.75,1), 
                    new ArmAutonPoint(this.arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z - 9).withTimeout(3.5), 
                    new PivotSetPoint(pivot, intakeAngle),
                    new IntakeBackward(intake)))
                //.andThen(Commands.parallel(new ArmAutonPoint(this.arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z - 2)), suck());//pickup 2nd piece
                .andThen(suck());
        }
        else {
            //drive to next piece and lower arm
            targetPoint = new Pose2d(startPosition.getX() + offsetX(4.5), startPosition.getY(), startPosition.getRotation());
            //pickup 2nd
            tempSequence = tempSequence//.andThen(new WaitCommand(0.5))
                .andThen(Commands.parallel(
                    new DriveToPoint(drive,odometry,targetPoint,1.5,1), 
                    new ArmAutonPoint(this.arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z - 11).withTimeout(3.5), 
                    new PivotSetPoint(pivot, intakeAngle),
                    new IntakeBackward(intake)))
                //.andThen(Commands.parallel(new ArmAutonPoint(this.arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z - 2)), suck());//pickup 2nd piece
                .andThen(suck());
        }
        
        //drive back into community
        targetPoint = new Pose2d(startPosition.getX() + offsetX(0.5), startPosition.getY(), startPosition.getRotation());
        tempSequence = tempSequence.andThen(Commands.parallel(new DriveToPoint(drive,odometry,targetPoint,1.75,1), new ArmAutonPoint(this.arm, Constants.ArmToScoreTopAuto_X, Constants.ArmToScoreTopAuto_Z - 4), new PivotSetPoint(pivot, 135)));
        
        //drive to scoring position
        //targetPoint = new Pose2d(startPosition.getX(), startPosition.getY(), startPosition.getRotation());
        tempSequence = tempSequence.andThen(new WaitCommand(0.5))//.andThen(new ArmAutonPoint(this.arm, Constants.ArmToScoreTopAuto_X, Constants.ArmToScoreTopAuto_Z - 4))
            .andThen(spit()); //score 2nd piece
        
        if(isClear){
            //try to put arm back over tail
            targetPoint = new Pose2d(startPosition.getX() + offsetX(4.55), startPosition.getY() + 1, startPosition.getRotation());
            tempSequence = tempSequence.andThen(Commands.parallel(new DriveToPoint(drive, odometry, targetPoint, 1.75, 1), new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z)));
        }
        else {
            targetPoint = new Pose2d(startPosition.getX() + offsetX(4.55), startPosition.getY(), startPosition.getRotation());
            tempSequence = tempSequence.andThen(Commands.parallel(new DriveToPoint(drive, odometry, targetPoint, 1.75, 1), new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z)));
        }
        return tempSequence;
    }

    public Command autoThreePiece(){
        Command tempSequence;
        Pose2d targetPoint;        
        //score 1st piece
        tempSequence = spit()
            .deadlineWith(new MoveWheelsSideways(drive));
        //drive to lane
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY() + 0.55, startPosition.getRotation());
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint,1.75,1));
        //drive to next piece and lower arm
        targetPoint = new Pose2d(startPosition.getX() + offsetX(4.55), startPosition.getY() + 0.55, startPosition.getRotation());
        //pickup 2nd
        tempSequence = tempSequence//.andThen(new WaitCommand(0.5))
            .andThen(Commands.parallel(new DriveToPoint(drive,odometry,targetPoint,1.75,1), new ArmAutonPoint(this.arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z - 1), new PivotSetPoint(pivot, 60)))
            //.andThen(Commands.parallel(new ArmAutonPoint(this.arm, Constants.ArmToPickupGround_X, Constants.ArmToPickupGround_Z - 2)), suck());//pickup 2nd piece
            .andThen(suck());
        //drive back into community
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY(), startPosition.getRotation());
        tempSequence = tempSequence.andThen(Commands.parallel(new DriveToPoint(drive,odometry,targetPoint,1.75,1), new ArmAutonPoint(this.arm, Constants.ArmToScoreTopAuto_X, Constants.ArmToScoreTopAuto_Z), new PivotSetPoint(pivot, 0)));
        
        //drive to scoring position
        //targetPoint = new Pose2d(startPosition.getX(), startPosition.getY(), startPosition.getRotation());
        tempSequence = tempSequence.andThen(spit()); //score 2nd piece

        //move back to lane
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY() + 0.55, startPosition.getRotation());
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint,0.75,1));
        
        //move to pieces
        targetPoint = new Pose2d(startPosition.getX() + offsetX(3.2), startPosition.getY() - 0.1, startPosition.getRotation().plus(Rotation2d.fromDegrees(offsetRot(225))));
        tempSequence = tempSequence.andThen(Commands.parallel(new DriveToPoint(drive,odometry,targetPoint,1.2,1.1)));//, //Running these two in parallel
        //TODO
        //(new ArmAutonPoint(this.arm, Constants.ArmToPickupGroundBack_X, Constants.ArmToPickupGroundBack_Z))));

        //move to next piece
        targetPoint = new Pose2d(startPosition.getX() + offsetX(4.008), startPosition.getY() - .824, startPosition.getRotation().plus(Rotation2d.fromDegrees(offsetRot(225))));
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint));

        //pickup 3rd piece
        tempSequence = tempSequence.andThen(suck());

        //move to lane
        targetPoint = new Pose2d(startPosition.getX() + offsetX(3.1), startPosition.getY() - 0.1, startPosition.getRotation().plus(Rotation2d.fromDegrees(offsetRot(-25))));
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint,0.85,1.76)); //TODO: implement speed and rot Modifiers

        //back to community
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY(), startPosition.getRotation());
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint,1.1,0.76)); //TODO: implement speed and rot Modifiers

        //drive to scoring position
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY() + -0.8, startPosition.getRotation());
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint,0.75,0.75)) //TODO: implement speed and rot Modifiers
            .andThen(new WaitCommand(0.5));
            //.andThen(new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z));
            //.andThen(new ArmAutonPoint(this.arm, Constants.ArmToScoreMiddle_X, Constants.ArmToScoreMiddle_Z));

        //score 3rd piece
        tempSequence = tempSequence.andThen(spit());

        //try to put arm back to tail
        //tempSequence = tempSequence.andThen(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));

        return tempSequence;
    }

    public Command autoPathPlanner() {
        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("DriveAroundStation", new PathConstraints(4, 3));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            odometry::getPose, // Pose2d supplier
            this::setStartPos, // Pose2d consumer, used to reset odometry at the beginning of auto
            drive.getKinematics(), // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            drive::setWheelCommand, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drive // The drive subsystem. Used to properly set the requirements of path following commands
        );

        return autoBuilder.fullAuto(pathGroup);
    }

    public Command autoPlanThreePiece() {
        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("3PieceLow", new PathConstraints(2, 3));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("armDown", new ArmAutonPoint(arm, Constants.ArmToPickupGroundBack_X, Constants.ArmToPickupGroundBack_Z));
        eventMap.put("armScore", new ArmAutonPoint(arm, Constants.ArmToPickupGroundBack_X, Constants.ArmToPickupGroundBack_Z));
        eventMap.put("spit", spit());
        eventMap.put("suck", suck());
        //eventMap.put("intakeDown", new IntakeDown());

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            odometry::getPose, // Pose2d supplier
            this::setStartPos, // Pose2d consumer, used to reset odometry at the beginning of auto
            drive.getKinematics(), // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            drive::setWheelCommand, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drive // The drive subsystem. Used to properly set the requirements of path following commands
        );

        return autoBuilder.fullAuto(pathGroup);
    }

    public Command autoThanksgiving() {
        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("ThanksgivingSurprise", new PathConstraints(1, 3));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("armDown", new ArmAutonPoint(arm, Constants.ArmToPickupGroundBack_X, Constants.ArmToPickupGroundBack_Z));
        eventMap.put("armScore", new ArmAutonPoint(arm, Constants.ArmToPickupGroundBack_X, Constants.ArmToPickupGroundBack_Z));
        eventMap.put("spit", spit());
        eventMap.put("suck", suck());
        //eventMap.put("intakeDown", new IntakeDown());

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            odometry::getPose, // Pose2d supplier
            this::setStartPos, // Pose2d consumer, used to reset odometry at the beginning of auto
            drive.getKinematics(), // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.7, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            drive::setWheelCommand, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drive // The drive subsystem. Used to properly set the requirements of path following commands
        );

        return autoBuilder.fullAuto(pathGroup);
    }

    public void setStartPos(Pose2d startPos) {
        if(DriverStation.getAlliance() == Alliance.Red) {
            //flip the orientation because Path Planner thinks the bottom left corner is always the starting position, but won't flip the field2d object
            Pose2d newPos = new Pose2d(15.98-startPos.getX(), 8.21-startPos.getY(),startPos.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
            startPosition = newPos;
        } else {
            startPosition = startPos;
        }
        odometry.resetPose(startPosition);
    }

    //START POSITION SETTER (FROM SHUFFLEBOARD INPUT)    
    public void setStartPos(){
        AutonomousStartPosition = startPosChooser.getSelected();
        alliance = DriverStation.getAlliance();
        if (alliance == DriverStation.Alliance.Blue){ //Start positions using smartdashboard, red 1-3, blue 1-3
            if(AutonomousStartPosition.equals(kNoObstacles)){
                startPosition = Constants.START_BLUE_LEFT;
            }
            else if(AutonomousStartPosition.equals(kBalance)){
                startPosition = Constants.START_BLUE_MIDDLE;
            }
            else if(AutonomousStartPosition.equals(kMobility)){
                startPosition = Constants.START_BLUE_MIDDLE;
            }
            else if(AutonomousStartPosition.equals(kMobilityBack)){
                startPosition = Constants.START_BLUE_MIDDLE;
            }
            else if(AutonomousStartPosition.equals(kCord)){
                startPosition = Constants.START_BLUE_RIGHT;
            } 
            else if(AutonomousStartPosition.equals(kOnePiece)){
                startPosition = Constants.START_BLUE_LEFT;
            }
            else if(AutonomousStartPosition.equals(kTwoPiece)){
                startPosition = Constants.START_BLUE_LEFT;
            }
            else if(AutonomousStartPosition.equals(kTwoPieceCord)){
                startPosition = Constants.START_BLUE_RIGHT;
            }
            else if(AutonomousStartPosition.equals(kThreePiece)){
                startPosition = Constants.START_BLUE_THREE_PIECE;
            }
            else if(AutonomousStartPosition.equals(kPathPlan)){
                startPosition = Constants.START_BLUE_RIGHT;
            }
            else if(AutonomousStartPosition.equals(kPlan3P)){
                startPosition = Constants.START_BLUE_LEFT;
            }
            else{
                SmartDashboard.putString("Error", "No Position");
            }
        }
        else if(alliance == DriverStation.Alliance.Red){
            if(AutonomousStartPosition.equals(kNoObstacles)){
                startPosition = Constants.START_RED_LEFT;
            }
            else if(AutonomousStartPosition.equals(kBalance)){
                startPosition = Constants.START_RED_MIDDLE;
            }
            else if(AutonomousStartPosition.equals(kMobility)){
                startPosition = Constants.START_RED_MIDDLE;
            }
            else if(AutonomousStartPosition.equals(kMobilityBack)){
                startPosition = Constants.START_RED_MIDDLE;
            }
            else if(AutonomousStartPosition.equals(kCord)){
                startPosition = Constants.START_RED_RIGHT;
            }
            else if(AutonomousStartPosition.equals(kOnePiece)){
                startPosition = Constants.START_RED_LEFT;
            }
            else if(AutonomousStartPosition.equals(kTwoPiece)){
                startPosition = Constants.START_RED_RIGHT;
            }
            else if(AutonomousStartPosition.equals(kTwoPieceCord)){
                startPosition = Constants.START_RED_LEFT;
            }
            else if(AutonomousStartPosition.equals(kThreePiece)){
                startPosition = Constants.START_RED_THREE_PIECE;
            }
            else if(AutonomousStartPosition.equals(kPathPlan)){
                startPosition = Constants.START_RED_RIGHT;
            }
            else if(AutonomousStartPosition.equals(kPlan3P)){
                startPosition = Constants.START_RED_LEFT;
            }
            else{
                SmartDashboard.putString("Error", "No Position");
            }
        }
        else{
            SmartDashboard.putString("Error", "No Team");
        }
    }

    //CUSTOM FUNCTIONS
    private Command suck(){
        Command tempCommand;
        //if(Robot.getGamePieceMode() == Robot.CUBE_MODE){
            tempCommand = new IntakeBackward(intake);
        // } else {
        //     tempCommand = new IntakeForward(intake);
        // }
        return tempCommand;
    }
    
    private Command spit(){
        Command tempCommand;
        // if(Robot.getGamePieceMode() == Robot.CONE_MODE){
        //     tempCommand = new IntakeBackward(intake);
        // } else {
        //     tempCommand = new IntakeForward(intake);
        // }
        tempCommand = (new IntakeForward(intake));
        return tempCommand;
    }

    private double offsetX(double offset){
        double result;
        if (alliance == DriverStation.Alliance.Red) {
            result = -offset;
        } else {
            result = offset;
        }
        return result;
    }

    private double offsetRot(double desRot){
        double result;
        if (alliance == DriverStation.Alliance.Red) {
            result = desRot;
        } else {
            result = desRot - 90;
        }
        return result;
    }
}
