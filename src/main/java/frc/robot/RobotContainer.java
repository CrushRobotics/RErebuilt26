// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    // Changed from FieldCentric to RobotCentric
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Elastic Dashboard Field Visualization
    private final Field2d field = new Field2d();

    public RobotContainer() {
        // Optional: Initialize DogLog options here if needed, e.g.:
        // DogLog.setOptions(new DogLogOptions()...);
        
        // Push the field object to SmartDashboard for Elastic
        SmartDashboard.putData("Field", field);

        // Add Swerve Drive State for Elastic (SwerveDrive Widget)
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
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Left Y (forward)
                    .withVelocityY(-joystick.getRightX() * MaxSpeed) // Strafe left/right with negative Right X
                    .withRotationalRate(-joystick.getLeftX() * MaxAngularRate) // Rotate with negative Left X
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        // Even in RobotCentric mode, this is useful for odometry reset, 
        // though it won't affect the 'forward' direction of driving anymore.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Register telemetry and update the Field2d object for Elastic
        drivetrain.registerTelemetry(state -> {
            logger.telemeterize(state);
            field.setRobotPose(state.Pose);
            
            // Update individual module poses for visualization (this creates the "icon map")
            var modulePoses = state.ModuleStates; // Or calculate actual poses if available
            // Note: Since CTRE 6 swerve state gives ModuleStates (velocity/angle) and ModulePositions (distance/angle),
            // visualizing them on Field2d usually requires constructing Pose2d objects for each module relative to the robot.
            // However, Elastic's Swerve Widget often just needs the ModuleStates array published to NT, which Telemetry.java handles.
            // If you specifically want them on the Field2d widget as icons:
            
            // Get module locations from constants to calculate their field pose
             var moduleLocations = drivetrain.getModuleLocations();
             var robotPose = state.Pose;
             
             for (int i = 0; i < moduleLocations.length; i++) {
                 var moduleLocation = moduleLocations[i];
                 var moduleState = state.ModuleStates[i];
                 
                 // Calculate module pose in field coordinates
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
        // ID 13 is "Outpost 1" on the Red Alliance side (from FieldConstants)
        int targetTagId = 13;

        return Commands.sequence(
            // 1. Reset pose to 0,0 at start of auto (useful for sim testing)
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            
            // 2. Generate trajectory and drive to the Outpost
            new DriveToAprilTag(drivetrain, targetTagId)
        );
    }
}