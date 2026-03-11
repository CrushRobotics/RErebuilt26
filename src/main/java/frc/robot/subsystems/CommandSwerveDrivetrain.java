package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private static final double kSimSteerGearRatio = 12.8;

    // SIM: manually tracked yaw integrated from chassis speeds
    private double m_simYawDeg = 0.0;

    // SIM: deadband to filter phantom omega — any rotation below this is ignored.
    // Set just above the phantom value so real commanded rotation still works.
    private static final double kSimOmegaDeadbandRadPerSec = 0.1;

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7),
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second),
            Volts.of(Math.PI),
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            setOperatorPerspectiveForward(
                alliance == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }

        DogLog.log("Sim/PoseYawDeg", getState().Pose.getRotation().getDegrees());
        DogLog.log("Sim/PigeonYawDeg", getPigeon2().getYaw().getValueAsDouble());
        DogLog.log("Sim/ChassisOmegaRadPerSec", getState().Speeds.omegaRadiansPerSecond);
        DogLog.log("Sim/SimYawDeg", m_simYawDeg);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        @SuppressWarnings("unchecked")
        SwerveModuleConstants<?, ?, ?>[] moduleConstants = new SwerveModuleConstants[] {
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        };

        for (int i = 0; i < 4; i++) {
            double offsetRotations = moduleConstants[i].EncoderOffset;
            var module = getModule(i);

            module.getEncoder().getSimState().setRawPosition(-offsetRotations);
            module.getSteerMotor().getSimState().setRawRotorPosition(-offsetRotations * kSimSteerGearRatio);
            module.getSteerMotor().getSimState().setRotorVelocity(0);
            module.getDriveMotor().getSimState().setRawRotorPosition(0);
            module.getDriveMotor().getSimState().setRotorVelocity(0);
        }

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            deltaTime = Math.min(deltaTime, 2 * kSimLoopPeriod);

            updateSimState(deltaTime, RobotController.getBatteryVoltage());

        
        double omega = getState().Speeds.omegaRadiansPerSecond;
        double correctedOmega = omega - 4.452;
        m_simYawDeg += Math.toDegrees(correctedOmega) * deltaTime;
        getPigeon2().getSimState().setRawYaw(m_simYawDeg);
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}