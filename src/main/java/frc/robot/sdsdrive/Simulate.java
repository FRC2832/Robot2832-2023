package frc.robot.sdsdrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

public class Simulate {
    private final double LOOP_TIME = 0.02;

    // robot parts
    private Drivetrain drive;
    private SwerveDriveKinematics kinematics;
    private SwerveModuleState[] swerveStates = new SwerveModuleState[4];

    // gyro simulation
    private double[] ypr_deg = new double[3];
    private double[] xyz_mps = new double[3];

    public Simulate(Drivetrain drive) {
        this.drive = drive;

        kinematics = drive.getKinematics();
        for (int i=0; i<swerveStates.length; i++) {
            swerveStates[i] = new SwerveModuleState();
        }
    }

    public void Init() {

    }


    public void Periodic() {
        if(DriverStation.isEnabled()) {
            updateDriveTrain();
        }
        else {
            xyz_mps[0] = 0;
            xyz_mps[1] = 0;
            for (SwerveModuleState swerveModuleState : swerveStates) {
                swerveModuleState.speedMetersPerSecond = 0;
            }
        }
    }

    private void updateDriveTrain() {
        SwerveModuleState[] moduleRequests = drive.getSwerveStateRequest();
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(moduleRequests);
        
        ypr_deg[0] += Math.toDegrees(speeds.omegaRadiansPerSecond * LOOP_TIME);
        xyz_mps[0] = speeds.vxMetersPerSecond / LOOP_TIME;
        xyz_mps[1] = speeds.vyMetersPerSecond / LOOP_TIME;

        //TODO: Simulate the actual swerve corners... https://www.chiefdelphi.com/t/sysid-gains-on-sds-mk4i-modules/400373/7
        for(int i=0; i<moduleRequests.length; i++) {
            swerveStates[i].speedMetersPerSecond = moduleRequests[i].speedMetersPerSecond;
            swerveStates[i].angle = moduleRequests[i].angle;
        }
    }

    public double[] getPigeonYpr() {
        return ypr_deg;
    }

    public double[] getPigeonXyz() {
        return xyz_mps;
    }

    public SwerveModuleState[] getSwerveStates() {
        return swerveStates;
    }
}
