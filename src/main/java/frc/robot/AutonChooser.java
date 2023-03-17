package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmAutonPoint;
import frc.robot.commands.DriveToBalance;
import frc.robot.commands.DriveToOffScale;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToScale;
import frc.robot.commands.DriveToScaleNegative;
import frc.robot.commands.IntakeBackward;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.MoveWheelsStraight;
import frc.robot.interfaces.ISwerveDrive;

public class AutonChooser {
    private Command sequence;
    private ISwerveDrive drive;
    private Intake intake;
    private Arm arm;
    private Odometry odometry;
    public Pose2d startPosition;
    private final SendableChooser<String> startPosChooser = new SendableChooser<>();
    private Alliance alliance;

    //Autonomous Chooser
    private static final String kNoObstacles = "No Obstacles";
    private static final String kBalance = "Scale";
    private static final String kCord = "Cord";
    private static final String kDoNothing = "Do Nothing";
    private static final String kOnePiece = "L3 Score";
    private static final String kTwoPiece = "Score Two";
    private static final String kThreePiece = "Score Three";

    private String AutonomousStartPosition;

    public AutonChooser(ISwerveDrive drive, Odometry odometry, Intake intake, Arm arm){
        this.drive = drive;
        this.odometry = odometry;
        this.intake = intake;
        this.arm = arm;

        sequence = new DriveToPoint(drive, odometry, odometry.getPose()); //if no sequence gets loaded it'll go to where it is (do nothing)

        startPosChooser.setDefaultOption("Scale", kBalance);
        startPosChooser.addOption("No Obstacles", kNoObstacles);
        startPosChooser.addOption("Cord", kCord);
        startPosChooser.addOption("Do Nothing", kDoNothing);
        startPosChooser.addOption("Score One", kOnePiece);
        startPosChooser.addOption("Score Two", kTwoPiece);
        startPosChooser.addOption("Score Three", kThreePiece);
        
        SmartDashboard.putData("StartPos Select", startPosChooser);

        setStartPos();
    }

    public Command getAuton(){
        if (AutonomousStartPosition.equals(kBalance)) {
            sequence = autoBalance();
        } else if (AutonomousStartPosition.equals(kNoObstacles)) {
            sequence = autoNoObstacles();
        } else if (AutonomousStartPosition.equals(kCord)) {
            sequence = autoCord();
        } else if (AutonomousStartPosition.equals(kOnePiece)){ //score on top row
            sequence = autoL3Score();
        } else if (AutonomousStartPosition.equals(kTwoPiece)){ //score two somewhere
            sequence = autoTwoPiece();
        } else if (AutonomousStartPosition.equals(kThreePiece)){ //score three somewhere
            sequence = autoThreePiece();
        } else {
            sequence = new MoveWheelsStraight(drive);
        }
        return sequence;
    }

    public Pose2d getStartPos(){
        return startPosition;
    }

    //ALL AUTON SEQUENCES
    public Command autoBalance(){ //drive over charge station out of community then come back in and balance - untested
        Command tempSequence;
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z)
            .deadlineWith(new MoveWheelsStraight(drive));
            
        tempSequence = tempSequence.andThen(spit());

        tempSequence = tempSequence.andThen(new WaitCommand(0.5)
            .andThen(new DriveToScale(drive)).withTimeout(2.9)
            .alongWith(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z))
            .andThen(new DriveToOffScale(drive))
            .andThen(new DriveToScaleNegative(drive))
            .andThen(new DriveToBalance(drive)).withTimeout(8));
        return tempSequence;
    }

    public Command autoNoObstacles(){
        Command tempSequence;
        Pose2d targetPoint;
        
        targetPoint = new Pose2d(startPosition.getX() + offsetX(4), startPosition.getY(), startPosition.getRotation());
            
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z)
            .deadlineWith(new MoveWheelsStraight(drive));
            
        tempSequence = tempSequence.andThen(spit());

        tempSequence = tempSequence.andThen(new WaitCommand(0.5)
            .andThen(new DriveToPoint(drive,odometry,targetPoint))
            .alongWith(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z)));
        return tempSequence;
    }

    public Command autoCord(){
        Command tempSequence;
        Pose2d targetPoint;
        
        targetPoint = new Pose2d(startPosition.getX() + offsetX(4), startPosition.getY(), startPosition.getRotation());
            
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z)
            .deadlineWith(new MoveWheelsStraight(drive));
        
        tempSequence = tempSequence.andThen(spit());
        
        tempSequence = tempSequence.andThen(new WaitCommand(0.5)
            .andThen(new DriveToPoint(drive,odometry,targetPoint))
            .alongWith(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z)));
        return tempSequence;
    }

    public Command autoL3Score(){
        Command tempSequence;
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z);
        tempSequence = tempSequence.andThen(spit());
        tempSequence = tempSequence.andThen(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));
        return tempSequence;
    }

    public Command autoTwoPiece(){
        Command tempSequence;
        Pose2d targetPoint;
        //Put arm in scoring position
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z)
            .deadlineWith(new MoveWheelsStraight(drive));
        
        //score 1st piece
        tempSequence = tempSequence.andThen(spit());
        
        //drive to next piece and lower arm
        targetPoint = new Pose2d(startPosition.getX() + offsetX(4), startPosition.getY() - 0.1, startPosition.getRotation().plus(Rotation2d.fromDegrees(180)));
        tempSequence = tempSequence.andThen(new WaitCommand(0.5))
            .andThen(new DriveToPoint(drive,odometry,targetPoint))
            .andThen(new ArmAutonPoint(this.arm, Constants.ArmToPickupGroundBack_X, Constants.ArmToPickupGroundBack_Z));
        
        //pickup 2nd piece
        tempSequence = tempSequence.andThen(suck());
        
        //drive back into community
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY(), startPosition.getRotation());
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint));
        
        //drive to scoring position
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY() + -0.36, startPosition.getRotation());
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint))
            .andThen(new WaitCommand(0.5))
            .andThen(new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z));
        
        //score 2nd piece
        tempSequence = tempSequence.andThen(spit());

        //try to put arm back over tail
        tempSequence = tempSequence.andThen(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));

        return tempSequence;
    }

    public Command autoThreePiece(){
        Command tempSequence;
        Pose2d targetPoint;
        //Put arm in scoring position
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z)
            .deadlineWith(new MoveWheelsStraight(drive));
        
        //score 1st piece
        tempSequence = tempSequence.andThen(spit());
        
        //drive to next piece and lower arm
        targetPoint = new Pose2d(startPosition.getX() + offsetX(4), startPosition.getY() - 0.1, startPosition.getRotation().plus(Rotation2d.fromDegrees(180)));
        tempSequence = tempSequence.andThen(new WaitCommand(0.5))
            .andThen(new DriveToPoint(drive,odometry,targetPoint))
            .andThen(new ArmAutonPoint(this.arm, Constants.ArmToPickupGroundBack_X, Constants.ArmToPickupGroundBack_Z));
        
        //pickup 2nd piece
        tempSequence = tempSequence.andThen(suck());
        
        //drive back into community
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY(), startPosition.getRotation());
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint));
        
        //drive to scoring position
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY() + -0.36, startPosition.getRotation());
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint))
            .andThen(new WaitCommand(0.5))
            .andThen(new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z));
        
        //score 2nd piece
        tempSequence = tempSequence.andThen(spit());

        //try to put arm back to pickup
        tempSequence = tempSequence.andThen(new ArmAutonPoint(this.arm, Constants.ArmToPickupGroundBack_X, Constants.ArmToPickupGroundBack_Z));

        //move back to lane
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY(), startPosition.getRotation());
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint));
        
        //move to pieces
        targetPoint = new Pose2d(startPosition.getX() + offsetX(4), startPosition.getY() - 0.1, startPosition.getRotation().plus(Rotation2d.fromDegrees(180)));
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint));

        //move to next piece
        targetPoint = new Pose2d(startPosition.getX() + offsetX(4), startPosition.getY() - 1.2, startPosition.getRotation().plus(Rotation2d.fromDegrees(180)));
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint));

        //pickup 3rd piece
        tempSequence = tempSequence.andThen(suck());

        //move to lane
        targetPoint = new Pose2d(startPosition.getX() + offsetX(4), startPosition.getY() - 0.1, startPosition.getRotation().plus(Rotation2d.fromDegrees(180)));
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint));

        //back to community
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY(), startPosition.getRotation());
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint));

        //drive to scoring position
        targetPoint = new Pose2d(startPosition.getX(), startPosition.getY() + -0.8, startPosition.getRotation());
        tempSequence = tempSequence.andThen(new DriveToPoint(drive,odometry,targetPoint))
            .andThen(new WaitCommand(0.5))
            .andThen(new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z));

        //score 3rd piece
        tempSequence = tempSequence.andThen(spit());

        //try to put arm back to tail
        tempSequence = tempSequence.andThen(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));

        return tempSequence;
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
            else if(AutonomousStartPosition.equals(kCord)){
                startPosition = Constants.START_BLUE_RIGHT;
            } 
            else if(AutonomousStartPosition.equals(kOnePiece)){
                startPosition = Constants.START_BLUE_LEFT;
            }
            else if(AutonomousStartPosition.equals(kTwoPiece)){
                startPosition = Constants.START_BLUE_LEFT;
            }
            else if(AutonomousStartPosition.equals(kThreePiece)){
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
            else if(AutonomousStartPosition.equals(kCord)){
                startPosition = Constants.START_RED_RIGHT;
            }
            else if(AutonomousStartPosition.equals(kOnePiece)){
                startPosition = Constants.START_RED_LEFT;
            }
            else if(AutonomousStartPosition.equals(kTwoPiece)){
                startPosition = Constants.START_RED_LEFT;
            }
            else if(AutonomousStartPosition.equals(kThreePiece)){
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
        if(Robot.getGamePieceMode() == Robot.CONE_MODE){
            tempCommand = new IntakeBackward(intake);
        } else {
            tempCommand = new IntakeForward(intake);
        }
        return tempCommand;
    }
    
    private Command spit(){
        Command tempCommand;
        if(Robot.getGamePieceMode() == Robot.CUBE_MODE){
            tempCommand = new IntakeBackward(intake);
        } else {
            tempCommand = new IntakeForward(intake);
        }
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
}
