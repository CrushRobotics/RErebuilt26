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

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); 

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle autoAimDrive = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    @SuppressWarnings("unused")
    private final VisionSubsystem vision = RobotBase.isReal() ? new VisionSubsystem(drivetrain) : null;
    
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
        });

        autoAimDrive.HeadingController.setPID(20.0, 0, 1.0);
        autoAimDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        autonomousLogic = new AutonomousLogic(drivetrain, hood, shooter, indexer);
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double rotAxis = RobotBase.isSimulation() ? joystick.getHID().getRawAxis(3) : joystick.getRightX();
                return drive.withVelocityX(-MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed)
                    .withVelocityY(-MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed)
                    .withRotationalRate(-MathUtil.applyDeadband(rotAxis, 0.1) * MaxAngularRate);
            })
        );

        // --- MANUAL SHOOTER TEST (RIGHT BUMPER) ---
        joystick.rightBumper().whileTrue(
            Commands.run(() -> {
                // Manually spin up the shooter to a set test velocity (e.g., 15 m/s)
                shooter.setTargetVelocity(15.0);
            }, shooter)
        ).onFalse(
            Commands.runOnce(() -> {
                // Stop the shooter when the bumper is released
                shooter.stop();
            }, shooter)
        );

        // --- BALLISTIC ALIGNMENT TEST (AXIS 4 TRIGGER) ---
        joystick.axisGreaterThan(4, 0.1).whileTrue(
            Commands.parallel(
                drivetrain.applyRequest(() -> {
                    double xVel = -MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed;
                    double yVel = -MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed;

                    Pose2d currentPose = drivetrain.getState().Pose;
                    Alliance alliance = DriverStation.getAlliance().orElse(RobotBase.isSimulation() ? Alliance.Blue : null);
                    
                    if (alliance == null) return drive.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(0);

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
                        DogLog.log("Shooter/TargetHeading", solution.chassisAimAngle.getDegrees());
                        return autoAimDrive.withVelocityX(xVel).withVelocityY(yVel)
                                           .withTargetDirection(solution.chassisAimAngle);
                    }
                    return drive.withVelocityX(xVel).withVelocityY(yVel).withRotationalRate(0);
                }),

                Commands.run(() -> {
                    FiringSolution sol = drivetrain.getCurrentFiringSolution(); 
                    if (sol != null) {
                        hood.setTargetAngle(sol.hoodAimAngle);
                        shooter.setTargetVelocity(sol.shotVelocityMps);
                        if (drivetrain.isChassisAimed(sol.chassisAimAngle)) {
                            indexer.feedAllBalls();
                        }
                    }
                }, hood, shooter, indexer)
            )
        ).onFalse(Commands.runOnce(() -> { shooter.stop(); indexer.stopFeeder(); }));

        // --- CALIBRATION BINDING ---
        // Pressing the "Back" button forces the robot to instantly calibrate to the starting position
        // This is a great way to "Establish where the robot is" without cameras during teleop testing.
        joystick.back().onTrue(drivetrain.runOnce(() -> {
            Pose2d startPose = new Pose2d(4.047, 0.629, Rotation2d.fromDegrees(-5.540));
            drivetrain.resetPose(startPose);
        }));
    }

    public void updateTelemetry() {
        try {
            logger.telemeterizeMechanisms(hood.getCurrentAngleDegrees(), indexer.getPositionRotations(), shooter.getPositionRotations(), climber.getExtensionMeters());
        } catch (Exception e) {}
    }

    public Command getAutonomousCommand() { return autonomousLogic.getSelectedAuto(); }
}