import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.BeforeAll;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveDriveTrain;

@TestInstance(Lifecycle.PER_CLASS)
public class SwerveTest {
    static final double DELTA = 1e-4; // acceptable deviation range
    SwerveModuleState request;
    SwerveModuleState[] requestArray;
    SwerveModuleState current;
    SwerveModuleState[] currentArray;
    SwerveModuleState[] result;

    @BeforeAll // this method will setup once
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    }

    @BeforeEach // this method will run before each test
    void setupEach() {
        //initialize data
        request = new SwerveModuleState(0,Rotation2d.fromDegrees(0));
        requestArray = new SwerveModuleState[] {request};
        current = new SwerveModuleState(0,Rotation2d.fromDegrees(0));
        currentArray = new SwerveModuleState[] {current};

        //swerves will change at max 0.2m/s per loop
        Preferences.setDouble(SwerveDriveTrain.MAX_ACCEL_KEY, 10);
        //swerves will change at max 2* per loop
        Preferences.setDouble(SwerveDriveTrain.MAX_OMEGA_KEY, 100);

        current.speedMetersPerSecond = 0;
        request.speedMetersPerSecond = 0;
        current.angle = Rotation2d.fromDegrees(0);
        request.angle = Rotation2d.fromDegrees(0);
    }

    @Test
    void SwerveFilters() {
        //check default forward
        current.speedMetersPerSecond = 0;
        request.speedMetersPerSecond = 5;
        result = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        assertEquals(0.2, result[0].speedMetersPerSecond, DELTA);

        //check at speed forward
        current.speedMetersPerSecond = 2;
        request.speedMetersPerSecond = 5;
        result = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        assertEquals(2.2, result[0].speedMetersPerSecond, DELTA);

        //check backwards going forward
        current.speedMetersPerSecond = -2;
        request.speedMetersPerSecond = 5;
        result = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        assertEquals(-1.8, result[0].speedMetersPerSecond, DELTA);

        //check backwards going backwards
        current.speedMetersPerSecond = -2;
        request.speedMetersPerSecond = -5;
        result = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        assertEquals(-2.2, result[0].speedMetersPerSecond, DELTA);

        //check small move backwards
        current.speedMetersPerSecond = -2;
        request.speedMetersPerSecond = -2.02;
        result = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        assertEquals(-2.02, result[0].speedMetersPerSecond, DELTA);

        //check stopped at zero
        current.speedMetersPerSecond = 0;
        request.speedMetersPerSecond = 0.02;
        result = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        assertEquals(0, result[0].speedMetersPerSecond, DELTA);

        //check stalled turn
        current.speedMetersPerSecond = 0;
        request.speedMetersPerSecond = 0.02;
        current.angle = Rotation2d.fromDegrees(0);
        request.angle = Rotation2d.fromDegrees(90);
        result = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        assertEquals(0, result[0].angle.getDegrees(), DELTA);

        //with turns, we need a little speed to get going
        current.speedMetersPerSecond = 1;
        request.speedMetersPerSecond = 1;

        //check left turn at angle
        current.angle = Rotation2d.fromDegrees(2);
        request.angle = Rotation2d.fromDegrees(90);
        result = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        assertEquals(4, result[0].angle.getDegrees(), DELTA);

        //check right turn at angle
        current.angle = Rotation2d.fromDegrees(-2);
        request.angle = Rotation2d.fromDegrees(-90);
        result = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        assertEquals(-4, result[0].angle.getDegrees(), DELTA);
    }

    //we want this to request past 180 for this loop just to get the hardware over, then the optomize will correct it to -180 to 180 range next loop
    @Test
    void SwerveLeftTurn180Cross() {
        current.speedMetersPerSecond = 1;
        request.speedMetersPerSecond = 1;
        current.angle = Rotation2d.fromDegrees(-179);
        request.angle = Rotation2d.fromDegrees(170);
        result = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        assertEquals(-181, result[0].angle.getDegrees(), DELTA);
    }

    @Test
    void SwerveLeftTurnGreater90() {
        //instead of going to 135*, the request should go to -45*
        current.speedMetersPerSecond = 1;
        request.speedMetersPerSecond = 1;
        current.angle = Rotation2d.fromDegrees(0);
        request.angle = Rotation2d.fromDegrees(135);
        for(int i=0; i<100;i++) {
            currentArray = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        }
        assertEquals(-1, currentArray[0].speedMetersPerSecond, DELTA);
        assertEquals(-45, currentArray[0].angle.getDegrees(), DELTA);
    }

    @Test
    void SwerveLeftTurnGreater90Offset() {
        current.speedMetersPerSecond = 1;
        request.speedMetersPerSecond = 1;
        current.angle = Rotation2d.fromDegrees(20);
        request.angle = Rotation2d.fromDegrees(155);
        for(int i=0; i<100;i++) {
            currentArray = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        }
        assertEquals(-1, currentArray[0].speedMetersPerSecond, DELTA);
        assertEquals(-25, currentArray[0].angle.getDegrees(), DELTA);
    }

    @Test
    void SwerveRightTurnGreater90() {
        //instead of going to -135*, the request should go to 45*
        current.speedMetersPerSecond = 1;
        request.speedMetersPerSecond = 1;
        current.angle = Rotation2d.fromDegrees(0);
        request.angle = Rotation2d.fromDegrees(-135);
        for(int i=0; i<100;i++) {
            currentArray = SwerveDriveTrain.optomizeSwerve(requestArray,currentArray);
        }
        assertEquals(-1, currentArray[0].speedMetersPerSecond, DELTA);
        assertEquals(45, currentArray[0].angle.getDegrees(), DELTA);
    }
}
