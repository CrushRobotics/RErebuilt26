package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.util.BallisticSolver;
import frc.robot.util.BallisticSolver.FiringSolution;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); 

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
    // Added back the heading controller for auto-aiming
    private final SwerveRequest.FieldCentricFacingAngle autoAimDrive = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.Velocity);
    
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    @SuppressWarnings("unused")
    private final VisionSubsystem vision = RobotBase.isReal() ? new VisionSubsystem(drivetrain) : null;
    
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    private final AutonomousLogic autonomousLogic;
    private final Field2d field = new Field2d();
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        SmartDashboard.putData("Field", field);
        
        drivetrain.registerTelemetry(state -> {
            field.setRobotPose(state.Pose);
            logger.telemeterize(state);
            DogLog.log("Drive/Pose", new Pose3d(state.Pose));
        });

        // Configure the PID for the auto-aim heading controller
        autoAimDrive.HeadingController.setPID(20.0, 0, 1.0);
        autoAimDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        autonomousLogic = new AutonomousLogic(drivetrain, hood, shooter, indexer);
        configureBindings();
    }

    private void configureBindings() {
        // --- DEFAULT DRIVE ---
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double rotAxis = RobotBase.isSimulation() ? joystick.getHID().getRawAxis(3) : joystick.getRightX();
                return drive.withVelocityX(-MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed)
                    .withVelocityY(-MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed)
                    .withRotationalRate(-MathUtil.applyDeadband(rotAxis, 0.1) * MaxAngularRate);
            })
        );

        // --- ADAPTIVE PRE-SPIN TOGGLE (B BUTTON) ---
        joystick.b().toggleOnTrue(
            Commands.run(() -> {
                Pose2d currentPose = drivetrain.getState().Pose;
                Alliance alliance = DriverStation.getAlliance().orElse(RobotBase.isSimulation() ? Alliance.Blue : null);
                
                if (alliance != null) {
                    Pose3d targetHub = (alliance == Alliance.Red) ? FieldConstants.RED_GOAL_POSE : FieldConstants.BLUE_GOAL_POSE;
                    var state = drivetrain.getState();
                    
                    FiringSolution solution = BallisticSolver.solveShot(
                        currentPose, 
                        targetHub, 
                        state.Speeds.vxMetersPerSecond, 
                        state.Speeds.vyMetersPerSecond, 
                        FieldConstants.ROBOT_SHOOTER_HEIGHT_METERS
                    );

                    if (solution != null) {
                        hood.setTargetAngle(Rotation2d.fromDegrees(solution.hoodAimAngle));
                        shooter.setTargetVelocity(solution.shotVelocityMps);
                        DogLog.log("Shooter/PreSpinActive", true);
                        
                        // Log exactly what the math is outputting so you can verify if the Solver is static
                        DogLog.log("Shooter/Solver_Raw_VelocityMPS", solution.shotVelocityMps);
                    }
                }
            }, hood, shooter)
            .finallyDo(() -> {
                shooter.stop();
                DogLog.log("Shooter/PreSpinActive", false);
            })
        );

        // --- AUTO-AIM / LOCK ONTO HUB (A BUTTON) ---
        // Holding A now calculates the shot, rotates the drivetrain to aim, AND revs the shooter/hood simultaneously.
        joystick.a().whileTrue(
            Commands.parallel(
                // 1. Drivetrain Aiming Logic
                drivetrain.applyRequest(() -> {
                    double xVel = -MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed;
                    double yVel = -MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed;

                    Pose2d currentPose = drivetrain.getState().Pose;
                    Alliance alliance = DriverStation.getAlliance().orElse(RobotBase.isSimulation() ? Alliance.Blue : null);
                    
                    if (alliance == null) return drive.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(0);

                    Pose3d targetHub = (alliance == Alliance.Red) ? FieldConstants.RED_GOAL_POSE : FieldConstants.BLUE_GOAL_POSE;
                    var state = drivetrain.getState();

                    FiringSolution solution = BallisticSolver.solveShot(
                        currentPose, targetHub, state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond, FieldConstants.ROBOT_SHOOTER_HEIGHT_METERS
                    );

                    if (solution != null) {
                        DogLog.log("Drive/TargetHeadingDegrees", solution.chassisAimAngle.getDegrees());
                        return autoAimDrive.withVelocityX(xVel).withVelocityY(yVel).withTargetDirection(solution.chassisAimAngle);
                    }
                    
                    return drive.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(0);
                }),
                
                // 2. Shooter & Hood Aiming Logic
                Commands.run(() -> {
                    Pose2d currentPose = drivetrain.getState().Pose;
                    Alliance alliance = DriverStation.getAlliance().orElse(RobotBase.isSimulation() ? Alliance.Blue : null);
                    
                    if (alliance != null) {
                        Pose3d targetHub = (alliance == Alliance.Red) ? FieldConstants.RED_GOAL_POSE : FieldConstants.BLUE_GOAL_POSE;
                        var state = drivetrain.getState();
                        
                        FiringSolution solution = BallisticSolver.solveShot(
                            currentPose, targetHub, state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond, FieldConstants.ROBOT_SHOOTER_HEIGHT_METERS
                        );

                        if (solution != null) {
                            hood.setTargetAngle(Rotation2d.fromDegrees(solution.hoodAimAngle));
                            shooter.setTargetVelocity(solution.shotVelocityMps);
                            
                            // Log exactly what the math is outputting
                            DogLog.log("Shooter/Solver_Raw_VelocityMPS", solution.shotVelocityMps);
                        }
                    }
                }, hood, shooter)
            )
        ).onFalse(Commands.runOnce(() -> shooter.stop(), shooter));

        // --- FIRE BUTTON (RIGHT BUMPER) ---
        joystick.rightBumper().whileTrue(
            Commands.run(() -> {
                indexer.feedAllBalls();
            }, indexer)
        ).onFalse(
            Commands.runOnce(() -> {
                indexer.stopFeeder();
            }, indexer)
        );

        // --- INTAKE PIVOT (LEFT BUMPER) ---
        joystick.leftBumper().whileTrue(
            Commands.run(() -> {
                // For testing with the 111:1 ratio, using the open-loop test method
                intake.setPivotSpeed(1.0); 
            }, intake)
        ).onFalse(
            Commands.runOnce(() -> {
                intake.stopAll(); 
            }, intake)
        );

        // --- INTAKE ROLLERS (X BUTTON TOGGLE) ---
        joystick.x().toggleOnTrue(
            Commands.run(() -> {
                intake.runIntakeRollers();
            }, intake)
            .finallyDo(() -> {
                intake.stopIntakeRollers();
            })
        );

        // --- CALIBRATION BINDING (BACK BUTTON) ---
        joystick.back().onTrue(drivetrain.runOnce(() -> {
            Pose2d startPose = new Pose2d(4.047, 0.629, Rotation2d.fromDegrees(-5.540));
            drivetrain.resetPose(startPose);
        }));
    }

    public void updateTelemetry() {
        try {
            logger.telemeterizeMechanisms(hood.getCurrentAngleDegrees(), indexer.getPositionRotations(), shooter.getPositionRotations(), climber.getExtensionMeters());
            
            // --- SIMULATOR CONTROLLER LOGGING ---
            DogLog.log("Controller/ButtonA_Held", joystick.getHID().getAButton());
            DogLog.log("Controller/ButtonB_Held", joystick.getHID().getBButton());
            DogLog.log("Controller/ButtonX_Held", joystick.getHID().getXButton());
            DogLog.log("Controller/RightBumper_Held", joystick.getHID().getRightBumper());
            DogLog.log("Controller/LeftBumper_Held", joystick.getHID().getLeftBumper());
            DogLog.log("Controller/BackButton_Held", joystick.getHID().getBackButton());
            
            DogLog.log("Controller/LeftY_Axis", joystick.getLeftY());
            DogLog.log("Controller/LeftX_Axis", joystick.getLeftX());
            DogLog.log("Controller/RightX_Axis", joystick.getRightX());
            
        } catch (Exception e) {}
    }

    public Command getAutonomousCommand() { return autonomousLogic.getSelectedAuto(); }
}