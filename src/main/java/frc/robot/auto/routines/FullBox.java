package frc.robot.auto.routines;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.FieldConstants;
import frc.robot.commands.AutoSmartGateCommand;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FullBox extends SequentialCommandGroup {

    public FullBox(CommandSwerveDrivetrain drivetrain, HoodSubsystem hood, ShooterSubsystem shooter, IndexerSubsystem indexer) {
        Pose2d startPose = new Pose2d(4.013, 7.255, Rotation2d.fromDegrees(171.935));

        // --- PHASE 1: Drive to Fuel ---
        List<Translation2d> phase1Waypoints = List.of(
            startPose.getTranslation(),      
            new Translation2d(0.832, 7.710), // WP1
            new Translation2d(0.372, 5.845), // WP2
            new Translation2d(0.378, 4.821), // WP3
            new Translation2d(2.356, 4.930), // WP4
            new Translation2d(5.067, 5.079)  // WP5 (Arriving at Centerline)
        );
        Rotation2d phase1Heading = Rotation2d.fromDegrees(34.648);

        // --- PHASE 2: Intake/Sweep ---
        List<Translation2d> phase2Waypoints = List.of(
            new Translation2d(5.067, 5.079), // WP5
            new Translation2d(6.610, 5.111), // WP6
            new Translation2d(7.849, 5.110)  // WP7 (End sweep)
        );
        // Fixed intake heading between 80-100 degrees
        Rotation2d phase2Heading = Rotation2d.fromDegrees(90.0);

        // --- PHASE 3: Drive Back (Refueling / Crossing Trench) ---
        List<Translation2d> phase3Waypoints = List.of(
            new Translation2d(7.849, 5.110), // WP7
            new Translation2d(7.955, 6.741), // WP8
            new Translation2d(6.240, 7.420)  // WP9 (Crossed Trench)
        );
        // Base heading set to Positive side (0.0). Intake faces balls, Shooter faces Hub.
        Rotation2d phase3Heading = Rotation2d.fromDegrees(0.0);

        // --- PHASE 4: Shooter Phase (Final Approach) ---
        List<Translation2d> phase4Waypoints = List.of(
            new Translation2d(6.240, 7.420), // WP9
            new Translation2d(4.636, 7.372)  // WP10
        );
        Rotation2d endPointHeading = Rotation2d.fromDegrees(-90.843);
        Pose2d targetPose = new Pose2d(1.090, 4.617, endPointHeading);

        addCommands(
            drivetrain.runOnce(() -> {
                drivetrain.resetPose(startPose);
                DogLog.log("Auto/TargetPose", targetPose);
            }),

            // Phase 1: Drive to Fuel (No dynamic aiming)
            drivetrain.getPurePursuitCommand(phase1Waypoints, phase1Heading, false),

            // Phase 2: Sweep Fuel at 90 Degrees
            // TODO: Wrap in Commands.parallel(drivetrain.get..., IntakeCommand)
            drivetrain.getPurePursuitCommand(phase2Waypoints, phase2Heading, false),

            // Phase 3 & 4: Drive back and Shoot simultaneously
            Commands.parallel(
                Commands.sequence(
                    // Phase 3: Escape centerline/Refuel. Dynamic Aiming TRUE keeps shooter on Hub.
                    drivetrain.getPurePursuitCommand(phase3Waypoints, phase3Heading, true),
                    // Phase 4: Final approach
                    drivetrain.getPurePursuitCommand(phase4Waypoints, endPointHeading, true),
                    new DriveToPose(drivetrain, targetPose)
                ),
                new AutoSmartGateCommand(
                    drivetrain, hood, shooter, indexer, 
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        return (alliance.isPresent() && alliance.get() == Alliance.Red) ? 
                               FieldConstants.RED_GOAL_POSE : FieldConstants.BLUE_GOAL_POSE;
                    },
                    () -> true
                )
            )
        );
    }
}