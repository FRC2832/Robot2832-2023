package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmAutonPoint;
import frc.robot.commands.DriveToBalance;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToScale;
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
    private static final String kL3Score = "L3 Score";
    private static final String kDoNothing = "Do Nothing";
    private String AutonomousStartPosition;

    public AutonChooser(ISwerveDrive drive, Odometry odometry, Intake intake, Arm arm){
        this.drive = drive;
        this.odometry = odometry;
        this.intake = intake;
        this.arm = arm;

        sequence = new DriveToPoint(drive, odometry, odometry.getPose()); //if no sequence gets loaded it'll go to where it is (do nothing)

        startPosChooser.addOption("No Obstacles", kNoObstacles);
        startPosChooser.setDefaultOption("Scale", kBalance);
        startPosChooser.addOption("Cord", kCord);
        startPosChooser.addOption("L3 Score Left", kL3Score);
        startPosChooser.addOption("Do Nothing", kDoNothing);

        SmartDashboard.putData("StartPos Select", startPosChooser);
        setStartPos();

        alliance = DriverStation.getAlliance();
    }

    public Command getAuton(){
        AutonomousStartPosition = startPosChooser.getSelected();

        if (AutonomousStartPosition.equals(kBalance)) {
            sequence = autoBalance();
        } else if (AutonomousStartPosition.equals(kNoObstacles) || AutonomousStartPosition.equals(kCord)) {
            sequence = autoNoObstacles();
        } else if (AutonomousStartPosition.equals(kL3Score)){ //score on top row
            sequence = autoL3Score();
        } else {
            sequence = new MoveWheelsStraight(drive);
        }
        return sequence;
    }

    public Pose2d getStartPos(){
        return startPosition;
    }

    public Command autoBalance(){
        Command tempSequence;
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z)
                .deadlineWith(new MoveWheelsStraight(drive));
        if(Robot.getGamePieceMode() == Robot.CUBE_MODE){
            tempSequence = tempSequence.andThen(new IntakeBackward(intake));
        } else {
            tempSequence = tempSequence.andThen(new IntakeForward(intake));
        }
        tempSequence = tempSequence.andThen(new WaitCommand(0.5).andThen(new DriveToScale(drive)).andThen(new DriveToBalance(drive))
            .alongWith(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z)));
        return tempSequence;
    }

    public Command autoNoObstacles(){
        Command tempSequence;
        Pose2d targetPoint;
        double offset;
        if (alliance == DriverStation.Alliance.Red) {
            offset = -4;
        } else {
            offset = 4;
        }
        targetPoint = new Pose2d(startPosition.getX() + offset, startPosition.getY(), startPosition.getRotation());
            
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z)
            .deadlineWith(new MoveWheelsStraight(drive));
        if(Robot.getGamePieceMode() == Robot.CUBE_MODE){
            tempSequence = tempSequence.andThen(new IntakeBackward(intake));
        } else {
            tempSequence = tempSequence.andThen(new IntakeForward(intake));
        }
        tempSequence = tempSequence.andThen(new WaitCommand(0.5).andThen(new DriveToPoint(drive,odometry,targetPoint))
            .alongWith(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z)));
        return tempSequence;
    }

    public Command autoCord(){
        Command tempSequence;
        Pose2d targetPoint;
        double offset;
        if (alliance == DriverStation.Alliance.Red) {
            offset = -4;
        } else {
            offset = 4;
        }
        targetPoint = new Pose2d(startPosition.getX() + offset, startPosition.getY(), startPosition.getRotation());
            
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z)
            .deadlineWith(new MoveWheelsStraight(drive));
        if(Robot.getGamePieceMode() == Robot.CUBE_MODE){
            tempSequence = tempSequence.andThen(new IntakeBackward(intake));
        } else {
            tempSequence = tempSequence.andThen(new IntakeForward(intake));
        }
        tempSequence = tempSequence.andThen(new WaitCommand(0.5).andThen(new DriveToPoint(drive,odometry,targetPoint))
            .alongWith(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z)));
        return tempSequence;
    }

    public Command autoL3Score(){
        Command tempSequence;
        tempSequence = new ArmAutonPoint(this.arm, Constants.ArmToScoreTop_X, Constants.ArmToScoreTop_Z);
        if(Robot.getGamePieceMode() == Robot.CUBE_MODE){
            tempSequence = tempSequence.andThen(new IntakeBackward(intake));
        } else {
            tempSequence = tempSequence.andThen(new IntakeForward(intake));
        }
        tempSequence = tempSequence.andThen(new ArmAutonPoint(this.arm, Constants.ArmToPickupTail_X, Constants.ArmToPickupTail_Z));
        return tempSequence;
    }

    public void setStartPos(){
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
            else if(AutonomousStartPosition.equals(kL3Score)){
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
            else if(AutonomousStartPosition.equals(kL3Score)){
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
}
