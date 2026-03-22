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
    // ... existing setup code ...
    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this)
    );

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    // --- BALLISTIC SOLVER INTEGRATION ---
    private FiringSolution currentFiringSolution = null;

    /**
     * Gathers robot state and calls the Ballistic Solver to find the required angles.
     */
    public FiringSolution calculateFiringSolution(Pose3d targetHub) {
        Pose2d pose = getState().Pose;
        ChassisSpeeds speeds = getState().Speeds;

        // If shooter is heavily offset from center, we could calculate tangential velocity here.
        // For now, we use standard chassis translational velocity.
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

    /**
     * Drivetrain gate check.
     */
    public boolean isChassisAimed(Rotation2d targetAngle) {
        return Math.abs(getState().Pose.getRotation().minus(targetAngle).getDegrees()) < 1.5
            && Math.abs(getState().Speeds.omegaRadiansPerSecond) < 3.0; 
    }

    // ... existing SysId & Sim methods ...
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) { return m_sysIdRoutineTranslation.quasistatic(direction); }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) { return m_sysIdRoutineTranslation.dynamic(direction); }
    public Command getPurePursuitCommand(List<Translation2d> path, Rotation2d heading) { return new PurePursuitCommand(this, path, heading); }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = Math.min(currentTime - m_lastSimTime, 2 * kSimLoopPeriod);
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    
    @Override 
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) { 
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds)); 
    }
}