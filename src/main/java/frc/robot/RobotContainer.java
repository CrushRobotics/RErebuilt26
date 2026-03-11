package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.DriveToAprilTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    private final Field2d field = new Field2d();

    public RobotContainer() {
        SmartDashboard.putData("Field", field);

        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> drivetrain.getState().ModuleStates[0].angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> drivetrain.getState().ModuleStates[0].speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> drivetrain.getState().ModuleStates[1].angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> drivetrain.getState().ModuleStates[1].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> drivetrain.getState().ModuleStates[2].angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> drivetrain.getState().ModuleStates[2].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> drivetrain.getState().ModuleStates[3].angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> drivetrain.getState().ModuleStates[3].speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> drivetrain.getState().Pose.getRotation().getRadians(), null);
            }
        });

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(state -> {
            logger.telemeterize(state);
            field.setRobotPose(state.Pose);

            var moduleLocations = drivetrain.getModuleLocations();
            var robotPose = state.Pose;

            for (int i = 0; i < moduleLocations.length; i++) {
                var moduleLocation = moduleLocations[i];
                var moduleState = state.ModuleStates[i];

                var modulePose = robotPose.transformBy(
                    new edu.wpi.first.math.geometry.Transform2d(
                        moduleLocation,
                        moduleState.angle
                    )
                );

                field.getObject("Module " + i).setPose(modulePose);
            }
        });
    }

    public Command getAutonomousCommand() {
        int targetTagId = 13;

        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            new DriveToAprilTag(drivetrain, targetTagId)
        );
    }
}