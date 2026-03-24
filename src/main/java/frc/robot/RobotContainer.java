package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.auto.AutonomousLogic;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.BallisticSolver.FiringSolution;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle autoAimDrive = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController joystick = new CommandXboxController(0);
    // TODO: Initialize Operator Controller for secondary mechanisms
    // private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // SAFETY: Suppress vision in sim if no coprocessor found
    @SuppressWarnings("unused")
    private final VisionSubsystem vision = RobotBase.isReal() ? new VisionSubsystem(drivetrain) : null;
    
    // Combat Mechanisms
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    // TODO: Initialize ClimberSubsystem once implemented
    private final ClimberSubsystem climber = new ClimberSubsystem();

    private final AutonomousLogic autonomousLogic;
    private final Field2d field = new Field2d();
    
    // RESTORED: Instantiate the CTRE Telemetry logger that feeds AdvantageScope
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        SmartDashboard.putData("Field", field);
        
        // RESTORED: Use the original CTRE Telemetry hook logic that worked previously
        drivetrain.registerTelemetry(state -> {
            // Update SmartDashboard Field
            field.setRobotPose(state.Pose);
            
            // RESTORED: Call the original CTRE Telemetry hook 
            // This natively formats data for AdvantageScope / SignalLogger
            logger.telemeterize(state);
            
            // ADDED: DogLog records for native AdvantageScope Swerve Module visualizers
            DogLog.log("Drive/Pose", state.Pose);
            DogLog.log("Drive/ModuleStates", state.ModuleStates);
            DogLog.log("Drive/ModuleTargets", state.ModuleTargets);

            // Log vision status to clear unused warning
            DogLog.log("Vision/EstimatorActive", vision != null);
        });

        autonomousLogic = new AutonomousLogic(drivetrain);
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                /* * ROTATION FIX: 
                 * User identified Axis 2 for turning in simulation.
                 */
                double rotAxis = RobotBase.isSimulation() ? joystick.getHID().getRawAxis(2) : joystick.getRightX();
                
                return drive.withVelocityX(-MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed)
                    .withVelocityY(-MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed)
                    .withRotationalRate(-MathUtil.applyDeadband(rotAxis, 0.1) * MaxAngularRate);
            })
        );

        // --- TRIPLE-CHECK SMART GATE AIMING (LEFT TRIGGER) ---
        joystick.leftTrigger().whileTrue(
            Commands.parallel(
                drivetrain.applyRequest(() -> {
                    double xVel = -MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed;
                    double yVel = -MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed;

                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isEmpty()) return drive.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(0);

                    var targetHub = (alliance.get() == Alliance.Red) ? 
                                    FieldConstants.RED_GOAL_POSE : FieldConstants.BLUE_GOAL_POSE;

                    FiringSolution solution = drivetrain.calculateFiringSolution(targetHub);

                    if (solution != null) {
                        return autoAimDrive.withVelocityX(xVel).withVelocityY(yVel)
                                           .withTargetDirection(solution.chassisAimAngle);
                    }
                    return drive.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(0);
                }),

                Commands.run(() -> {
                    FiringSolution solution = drivetrain.getCurrentFiringSolution();
                    if (solution == null) {
                        indexer.stopFeeder();
                        return;
                    }

                    hood.setTargetAngle(solution.hoodAimAngle);
                    shooter.setTargetVelocity(solution.shotVelocityMps);

                    if (drivetrain.isChassisAimed(solution.chassisAimAngle) && 
                        hood.isAtAngle(solution.hoodAimAngle) && 
                        shooter.isAtVelocity(solution.shotVelocityMps)) {
                        indexer.feedAllBalls();
                    } else {
                        indexer.stopFeeder();
                    }
                }, hood, shooter, indexer)
            )
        ).onFalse(
            Commands.runOnce(() -> {
                shooter.stop();
                indexer.stopFeeder();
            })
        );

        // TODO: Map bindings for intaking game pieces
        // TODO: Map bindings for the climber subsystem
    }

    /** * Called periodically to update the 3D mechanisms in telemetry. */
    public void updateTelemetry() {
        try {
            logger.telemeterizeMechanisms(
                hood.getCurrentAngleDegrees(),
                indexer.getPositionRotations(),
                shooter.getPositionRotations(),
                climber.getExtensionMeters()
            );
        } catch (Exception e) {}
    }

    public Command getAutonomousCommand() { return autonomousLogic.getSelectedAuto(); }
}