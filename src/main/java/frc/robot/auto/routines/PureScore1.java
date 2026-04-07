package frc.robot.auto.routines;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.FieldConstants;
import frc.robot.commands.AutoSmartGateCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PureScore1 extends SequentialCommandGroup {

    public PureScore1(CommandSwerveDrivetrain drivetrain, HoodSubsystem hood, ShooterSubsystem shooter, IndexerSubsystem indexer) {
        // Start Pose
        Pose2d startPose = new Pose2d(4.006, 0.679, Rotation2d.fromDegrees(-5.540));
        
        // --- PHASE 1: Drive to Fuel ---
        List<Translation2d> phase1Waypoints = List.of(
            startPose.getTranslation(),      
            new Translation2d(6.670, 0.679)  // WP1
        );
        Rotation2d phase1Heading = Rotation2d.fromDegrees(0.0);

        // --- PHASE 2: Intake/Sweep ---
        List<Translation2d> phase2Waypoints = List.of(
            new Translation2d(6.670, 0.679), // WP1
            new Translation2d(7.780, 1.766), // WP2
            new Translation2d(7.785, 3.168), // WP3
            new Translation2d(7.790, 4.711)  // WP4 (End sweep)
        );
        Rotation2d phase2Heading = Rotation2d.fromDegrees(90.0); 

        // --- PHASE 3: Drive Back (Crossing Trench) ---
        List<Translation2d> phase3Waypoints = List.of(
            new Translation2d(7.790, 4.711), // WP4
            new Translation2d(7.301, 1.083), // WP5
            new Translation2d(4.006, 0.679)  // WP6 (Crossed Trench)
        );
        // SMOOTH ALIGNMENT: Point to our side (180 degrees) instead of jittery dynamic aiming.
        Rotation2d phase3Heading = Rotation2d.fromDegrees(180.0);

        // --- PHASE 4: Stop AND REFUEL (Auto Aim Starts Here) ---
        List<Translation2d> phase4Waypoints = List.of(
            new Translation2d(4.006, 0.679), // WP6
            new Translation2d(0.513, 0.675)  // WP7 (Depot at the back wall)
        );
        
        // --- PHASE 5: Climb Approach (End Point) ---
        List<Translation2d> phase5Waypoints = List.of(
            new Translation2d(0.513, 0.675), // WP7 (Depot)
            new Translation2d(2.254, 2.073)  // End Point
        );

        // A dummy heading. When enableDynamicAiming is TRUE, this is completely ignored.
        Rotation2d dummyHeading = Rotation2d.kZero;

        // NEW: Create a Trapezoidal Profiled PID Controller for buttery smooth rotations.
        ProfiledPIDController aimController = new ProfiledPIDController(
            10.0, 0, 0.1, 
            new TrapezoidProfile.Constraints(
                Math.PI * 2.5, // Max velocity (rad/s) -> extremely fast spin
                Math.PI * 6.0  // Max acceleration (rad/s^2) -> reaches max velocity smoothly in ~0.4s
            )
        );
        aimController.enableContinuousInput(-Math.PI, Math.PI);

        // We use a basic FieldCentric request and manually pass the calculated rotational rate
        SwerveRequest.FieldCentric aimHoldRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

        addCommands(
            drivetrain.runOnce(() -> {
                drivetrain.resetPose(startPose);
                DogLog.log("Auto/TargetPose", new Pose2d(2.254, 2.073, Rotation2d.kZero));
            }),

            // Phase 1: Drive to Fuel (No dynamic aiming)
            drivetrain.getPurePursuitCommand(phase1Waypoints, phase1Heading, false),

            // Phase 2: Sweep Fuel at 90 Degrees
            drivetrain.getPurePursuitCommand(phase2Waypoints, phase2Heading, false),

            // Phase 3: Travel Back (No shooting here anymore, just get to WP6 smoothly)
            drivetrain.getPurePursuitCommand(phase3Waypoints, phase3Heading, false),

            // --- PHASES 4 and 5: Precision Depot Approach & Auto Aim ---
            Commands.deadline(
                Commands.sequence(
                    // --> START AUTO AIM AT WP6 <--
                    // 1) Reset the PID profile to the robot's current angle so it doesn't violently jerk on start
                    Commands.runOnce(() -> aimController.reset(drivetrain.getState().Pose.getRotation().getRadians())),

                    // 2) ACTIVE Aim Hold at WP6 to shoot the swept balls.
                    Commands.run(() -> {
                        Rotation2d aimAngle = dummyHeading;
                        if (drivetrain.getCurrentFiringSolution() != null) {
                            aimAngle = drivetrain.getCurrentFiringSolution().chassisAimAngle;
                        }
                        
                        // Calculate smooth rotational rate from the profile
                        double rotRate = aimController.calculate(
                            drivetrain.getState().Pose.getRotation().getRadians(),
                            aimAngle.getRadians()
                        );
                        
                        drivetrain.setControl(
                            aimHoldRequest
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(rotRate)
                        );
                    }, drivetrain).withTimeout(0.8),

                    // Phase 4: Drive to Depot (Dynamic aiming stays locked on target)
                    drivetrain.getPurePursuitCommand(phase4Waypoints, dummyHeading, true),
                    
                    // 1) Reset the PID profile to the robot's current angle again
                    Commands.runOnce(() -> aimController.reset(drivetrain.getState().Pose.getRotation().getRadians())),

                    // 2) ACTIVE Aim Hold at the depot to catch human player balls.
                    Commands.run(() -> {
                        Rotation2d aimAngle = dummyHeading;
                        if (drivetrain.getCurrentFiringSolution() != null) {
                            aimAngle = drivetrain.getCurrentFiringSolution().chassisAimAngle;
                        }
                        
                        // Calculate smooth rotational rate from the profile
                        double rotRate = aimController.calculate(
                            drivetrain.getState().Pose.getRotation().getRadians(),
                            aimAngle.getRadians()
                        );
                        
                        drivetrain.setControl(
                            aimHoldRequest
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(rotRate)
                        );
                    }, drivetrain).withTimeout(1.5),
                    
                    // Final strafe to the climber rung
                    drivetrain.getPurePursuitCommand(phase5Waypoints, dummyHeading, true)
                ), 
                
                // This gate command runs continuously from WP6 onwards.
                new AutoSmartGateCommand(
                    drivetrain, hood, shooter, indexer, 
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        return (alliance.isPresent() && alliance.get() == Alliance.Red) ? 
                               FieldConstants.RED_GOAL_POSE : FieldConstants.BLUE_GOAL_POSE;
                    },
                    () -> true
                )
            ),

            // --- FINAL CLEANUP: STOP THE MOTORS! ---
            Commands.runOnce(() -> {
                drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
                shooter.stop();
                indexer.stopFeeder();
            })
        );
    }
}