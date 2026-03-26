package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
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
    
    // This request uses CTRE's internal closed-loop PID to snap to a specific heading while driving
    private final SwerveRequest.FieldCentricFacingAngle autoAimDrive = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    @SuppressWarnings("unused")
    private final VisionSubsystem vision = RobotBase.isReal() ? new VisionSubsystem(drivetrain) : null;
    
    // Combat Mechanisms
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();

    private final AutonomousLogic autonomousLogic;
    private final Field2d field = new Field2d();
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        SmartDashboard.putData("Field", field);
        
        drivetrain.registerTelemetry(state -> {
            field.setRobotPose(state.Pose);
            logger.telemeterize(state);
            
            DogLog.log("Drive/Pose", new Pose3d(state.Pose));
            DogLog.log("Drive/ModuleStates", state.ModuleStates);
            DogLog.log("Drive/ModuleTargets", state.ModuleTargets);
            DogLog.log("Vision/EstimatorActive", vision != null);
        });

        autonomousLogic = new AutonomousLogic(drivetrain, hood, shooter, indexer);
        configureBindings();
    }

    private void configureBindings() {
        // --- DEFAULT DRIVE ---
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // Manual rotation axis moved to Axis 3 (Right X) to free up Axis 4 for the trigger
                double rotAxis = RobotBase.isSimulation() ? joystick.getHID().getRawAxis(3) : joystick.getRightX();
                
                SmartDashboard.putNumber("Debug/RotAxis", rotAxis);

                return drive.withVelocityX(-MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed)
                    .withVelocityY(-MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed)
                    .withRotationalRate(-MathUtil.applyDeadband(rotAxis, 0.1) * MaxAngularRate);
            })
        );

        // --- DYNAMIC AUTO-AIM (TRIGGERED BY AXIS 4) ---
        // Using axisGreaterThan(4, 0.1) to treat the analog axis 4 as a button trigger
        joystick.axisGreaterThan(4, 0.1).whileTrue(
            Commands.parallel(
                // 1. Drivetrain control: Translates with left stick, rotation hijacked for Hub
                drivetrain.applyRequest(() -> {
                    double xVel = -MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed;
                    double yVel = -MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed;

                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    
                    // Simulation Fix: Default to Blue alliance if not set in Sim GUI
                    Alliance currentAlliance = alliance.orElse(RobotBase.isSimulation() ? Alliance.Blue : null);
                    
                    if (currentAlliance == null) {
                        return drive.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(0);
                    }

                    var targetHub = (currentAlliance == Alliance.Red) ? 
                                    FieldConstants.RED_GOAL_POSE : FieldConstants.BLUE_GOAL_POSE;

                    FiringSolution solution = drivetrain.calculateFiringSolution(targetHub);

                    // If we have a solution, snap to it. Otherwise, stay oriented as we are.
                    if (solution != null) {
                        return autoAimDrive.withVelocityX(xVel).withVelocityY(yVel)
                                           .withTargetDirection(solution.chassisAimAngle);
                    }
                    
                    // Fallback to manual rotation via Axis 3 if the ballistic solver fails to find a shot
                    double rotAxis = RobotBase.isSimulation() ? joystick.getHID().getRawAxis(3) : joystick.getRightX();
                    return drive.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(-MathUtil.applyDeadband(rotAxis, 0.1) * MaxAngularRate);
                }),

                // 2. Shooter/Hood/Indexer control
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
    }

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