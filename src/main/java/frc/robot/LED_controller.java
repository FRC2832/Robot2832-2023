package frc.robot;

import org.livoniawarriors.Logger;
import org.livoniawarriors.REVDigitBoard;

import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.leds.BalanceLeds;
import frc.robot.commands.leds.BreathLeds;
import frc.robot.commands.leds.FillLeds;
import frc.robot.commands.leds.LightningFlash;
import frc.robot.commands.leds.PinkWave;
import frc.robot.commands.leds.TargetLeds;
import frc.robot.commands.leds.TestLeds;
import frc.robot.interfaces.IDriveControls;
import frc.robot.interfaces.IOperatorControls;
import frc.robot.interfaces.ISwerveDrive;

public class LED_controller {
    public static final String kNormalLeds = "Normal";
    public static final String kPinkLeds = "Pink";
 
    public static final int PinkHue = 158;
    public static final int GreenHue = 60;
    public static final int RedHue = 0;
    public static final int BlueHue = 120;
    public static final int ConeHue = 17;
    public static final int CubeHue = 136;

    private REVDigitBoard digit;
    private boolean lastPieceMode;
    private int rumbleCounts;
    private ArmLedSubsystem leds;
    private TargetLedSubsystem targetLeds;
    private static SendableChooser<String> m_ledPattern;

    public LED_controller(){
        digit = new REVDigitBoard();
        lastPieceMode = false;

        m_ledPattern = new SendableChooser<>();
        m_ledPattern.addOption(kNormalLeds, kNormalLeds);
        m_ledPattern.setDefaultOption(kPinkLeds, kPinkLeds);
        SmartDashboard.putData("Color Scheme", m_ledPattern);

        //new LED controller
        leds = new ArmLedSubsystem();
        Command defaultCmd = new PinkWave(leds);
        leds.setDefaultCommand(defaultCmd);
        defaultCmd.schedule();

        //start the targetting animation
        targetLeds = new TargetLedSubsystem(leds);
        defaultCmd = new TargetLeds(targetLeds);
        targetLeds.setDefaultCommand(defaultCmd);
        defaultCmd.schedule();

        //put button to try leds on the dashboard
        SmartDashboard.putData(new TestLeds(leds));
        SmartDashboard.putData(new LightningFlash(leds,PinkHue));
    }

    private enum Mode {
        kNone,
        kDisabled,
        kAutonomous,
        kTeleop,
        kTest
    }

    boolean lastSpit;
    DSControlWord m_word = new DSControlWord();
    Mode lastMode = Mode.kNone;
    public void update(ISwerveDrive drive, Intake intake, Tail tail, IDriveControls driver, IOperatorControls opControls) {
        //check for rumble
        var rumble = 0.;
        if(intake.HasPiece() || tail.HasPiece()) {
            if(rumbleCounts < 40) {
                rumble = 0.8;
            }
            rumbleCounts++;
        } else {
            rumbleCounts = 0;
        }
        driver.SetRumble(rumble);
        opControls.SetRumble(rumble);

        
        // Get current mode
        m_word.refresh();
        Mode mode = Mode.kNone;
        if (m_word.isDisabled()) {
            mode = Mode.kDisabled;
        } else if (m_word.isAutonomous()) {
            mode = Mode.kAutonomous;
        } else if (m_word.isTeleop()) {
            mode = Mode.kTeleop;
        } else if (m_word.isTest()) {
            mode = Mode.kTest;
        }

        //check if robot mode changed
        if(!mode.equals(lastMode)) {
            Command newCommand;
            //there was a change, switch default led pattern
            if(mode == Mode.kAutonomous) {
                //autonomous
                newCommand = new BalanceLeds(leds, drive::getPitch);
            } else if (mode == Mode.kTeleop) {
                //teleop pattern
                newCommand = new SequentialCommandGroup(
                    new FillLeds(leds),
                    new BreathLeds(leds)
                );
            } else {
                //disabled
                newCommand = new PinkWave(leds);
            }
            leds.setDefaultCommand(newCommand);
            newCommand.schedule();
        }
        lastMode = mode;

        //check if we started spitting to run the animation
        boolean spit = opControls.IntakeSpitRequested().getAsBoolean();
        if(lastSpit == false && spit == true) {
            new LightningFlash(leds,GreenHue).schedule();
        }
        lastSpit = spit;

        //if we started getting a piece, flash the leds to let the drive know
        if (rumbleCounts == 1) {
            if(DriverStation.getAlliance() == Alliance.Red) {
                new LightningFlash(leds,RedHue).schedule();
            } else {
                new LightningFlash(leds,BlueHue).schedule();
            }
        } 

        //run the digit board
        if(Logger.FaultSet()) {
            digit.display(" FLT");
        } else if (Logger.StickyFaultSet()) {
            digit.display("SFLT");
        } else {
            digit.display("RONY");
        }

        //check if piece mode needs to be switched
        String pieceMode;
        var newMode = opControls.ChangePieceMode().getAsBoolean();
        boolean currentMode = Robot.getGamePieceMode();
        if(newMode == true && lastPieceMode == false) {
            Robot.setGamePieceMode(!currentMode);
        }
        if(currentMode == Robot.CONE_MODE) {
            pieceMode = "Cone!";
        } else {
            pieceMode = "Cube!";
        }
        SmartDashboard.putString("Piece Mode", pieceMode);
        lastPieceMode = newMode;
    }

    public static String getLedMode() {
        String mode = m_ledPattern.getSelected();
        if(mode == null) {
            return kNormalLeds;
        }
        return mode;
    }
}
