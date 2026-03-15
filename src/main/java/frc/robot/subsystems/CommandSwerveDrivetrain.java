package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.commands.PurePursuitCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * The drivetrain subsystem for the robot.
 * Extends the generated TunerSwerveDrivetrain to add autonomous and SysId support.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // Request used for Choreo/Auto trajectories
    private final SwerveRequest.ApplyRobotSpeeds m_autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    // SysId Characterization Requests
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this)
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null, state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> {
            setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
        }, null, this)
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) startSimThread();
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    /**
     * SysId control methods for characterization.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /**
     * Factory for Choreo paths.
     * @param trajectoryName Name of the .traj file in deploy/choreo
     */
    public Command getChoreoCommand(String trajectoryName) {
        // Explicitly typed for the 2026 ChoreoLib API
        Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory(trajectoryName);
        
        if (trajectory.isEmpty()) {
            DriverStation.reportError("Choreo: Trajectory '" + trajectoryName + "' not found!", false);
            return Commands.none();
        }

        return Choreo.choreoSwerveCommand(
            trajectory.get(), 
            () -> getState().Pose, 
            new PIDController(5.0, 0.0, 0.0), // X PID
            new PIDController(5.0, 0.0, 0.0), // Y PID
            new PIDController(5.0, 0.0, 0.0), // Rotation PID
            (ChassisSpeeds speeds) -> setControl(m_autoRequest.withSpeeds(speeds)), 
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == Alliance.Red;
            }, 
            this
        );
    }

    /**
     * Factory for Pure Pursuit paths.
     */
    public Command getPurePursuitCommand(List<Translation2d> path, Rotation2d heading) {
        return new PurePursuitCommand(this, path, heading);
    }

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

    @Override public void periodic() {
        DogLog.log("Sim/PoseDegrees", getState().Pose.getRotation().getDegrees());
    }
    
    @Override 
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) { 
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds)); 
    }
}