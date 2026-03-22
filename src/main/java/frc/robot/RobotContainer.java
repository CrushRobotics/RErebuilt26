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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.auto.AutonomousLogic;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.BallisticSolver.FiringSolution;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle autoAimDrive = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    
    // Combat Mechanisms
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final IndexerSubsystem indexer = new IndexerSubsystem();

    private final AutonomousLogic autonomousLogic;
    private final Field2d field = new Field2d();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        SmartDashboard.putData("Field", field);
        autonomousLogic = new AutonomousLogic(drivetrain);
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double rotAxis = RobotBase.isSimulation() ? joystick.getHID().getRawAxis(2) : joystick.getRightX();
                return drive.withVelocityX(-MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed)
                    .withVelocityY(-MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed)
                    .withRotationalRate(-MathUtil.applyDeadband(rotAxis, 0.1) * MaxAngularRate);
            })
        );

        // --- TRIPLE-CHECK SMART GATE AIMING (LEFT TRIGGER) ---
        joystick.leftTrigger().whileTrue(
            Commands.parallel(
                // Task A: Update Drivetrain Yaw
                drivetrain.applyRequest(() -> {
                    double xVel = -MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed;
                    double yVel = -MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed;

                    // Get target based on Alliance
                    var alliance = DriverStation.getAlliance();
                    var targetHub = (alliance.isPresent() && alliance.get() == Alliance.Red) ? 
                                    FieldConstants.RED_GOAL_POSE : FieldConstants.BLUE_GOAL_POSE;

                    // Fetch the 3D physics solution!
                    FiringSolution solution = drivetrain.calculateFiringSolution(targetHub);

                    if (solution != null) {
                        return autoAimDrive.withVelocityX(xVel).withVelocityY(yVel)
                                           .withTargetDirection(solution.chassisAimAngle);
                    }
                    // Fallback if no mathematical solution exists (e.g. directly underneath)
                    return drive.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(0);
                }),

                // Task B: Update Mechanisms & Control Gate
                Commands.run(() -> {
                    FiringSolution solution = drivetrain.getCurrentFiringSolution();
                    if (solution == null) {
                        indexer.stopFeeder();
                        return;
                    }

                    // 1. Move Mechanisms
                    hood.setTargetAngle(solution.hoodAimAngle);
                    shooter.setTargetVelocity(solution.shotVelocityMps);

                    // 2. TRIPLE GATE CHECK
                    boolean isChassisReady = drivetrain.isChassisAimed(solution.chassisAimAngle);
                    boolean isHoodReady = hood.isAtAngle(solution.hoodAimAngle);
                    boolean isShooterReady = shooter.isAtVelocity(solution.shotVelocityMps);

                    // 3. Fire only if all 3 are locked!
                    if (isChassisReady && isHoodReady && isShooterReady) {
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

    public Command getAutonomousCommand() { return autonomousLogic.getSelectedAuto(); }
}