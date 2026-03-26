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

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    // --- BALLISTIC SOLVER INTEGRATION ---
    private FiringSolution currentFiringSolution = null;

    public FiringSolution calculateFiringSolution(Pose3d targetHub) {
        var state = getState();
        Pose2d pose = state.Pose;
        
        // FIX: Convert Robot-Relative speeds to Field-Relative for the solver
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            state.Speeds, 
            pose.getRotation()
        );

        FiringSolution solution = BallisticSolver.solveShot(
            pose, 
            targetHub, 
            fieldSpeeds.vxMetersPerSecond, 
            fieldSpeeds.vyMetersPerSecond, 
            FieldConstants.ROBOT_SHOOTER_HEIGHT_METERS
        );

        this.currentFiringSolution = solution;
        return solution;
    }

    public FiringSolution getCurrentFiringSolution() {
        return currentFiringSolution;
    }

    public boolean isChassisAimed(Rotation2d targetAngle) {
        return Math.abs(getState().Pose.getRotation().minus(targetAngle).getDegrees()) < 1.5;
    }

    public Command getPurePursuitCommand(List<Translation2d> path, Rotation2d heading, boolean enableDynamicAiming) { 
        return new PurePursuitCommand(this, path, heading, enableDynamicAiming); 
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}