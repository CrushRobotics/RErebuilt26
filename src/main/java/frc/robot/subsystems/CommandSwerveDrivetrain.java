package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.FieldConstants;
import frc.robot.commands.PurePursuitCommand;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.BallisticSolver;
import frc.robot.util.BallisticSolver.FiringSolution;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    
    private static final double kSimLoopPeriod = 0.005; // 5ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this)
    );

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        // --- RESTORED ORIGINAL CTRE CONSTRUCTOR ---
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();
    }

    /**
     * applyRequest command to use SwerveRequests via command-based framework.
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    // --- BALLISTIC SOLVER INTEGRATION ---
    private FiringSolution currentFiringSolution = null;

    public FiringSolution calculateFiringSolution(Pose3d targetHub) {
        Pose2d pose = getState().Pose;
        ChassisSpeeds speeds = getState().Speeds;

        FiringSolution solution = BallisticSolver.solveShot(
            pose, 
            targetHub, 
            speeds.vxMetersPerSecond, 
            speeds.vyMetersPerSecond, 
            FieldConstants.ROBOT_SHOOTER_HEIGHT_METERS
        );

        this.currentFiringSolution = solution;
        return solution;
    }

    public FiringSolution getCurrentFiringSolution() {
        return currentFiringSolution;
    }

    public boolean isChassisAimed(Rotation2d targetAngle) {
        return Math.abs(getState().Pose.getRotation().minus(targetAngle).getDegrees()) < 1.5
            && Math.abs(getState().Speeds.omegaRadiansPerSecond) < 3.0; 
    }

    // --- REMOVED THE BREAKING PERIODIC OVERRIDE ---
    // The super.periodic() is required to copy simulated data into the robot Pose.

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) { return m_sysIdRoutineTranslation.quasistatic(direction); }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) { return m_sysIdRoutineTranslation.dynamic(direction); }
    public Command getPurePursuitCommand(List<Translation2d> path, Rotation2d heading) { return new PurePursuitCommand(this, path, heading); }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            
            /* Feed physics to sim */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    
    @Override 
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) { 
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds)); 
    }
}