package frc.robot.auto.routines;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PureScore2 extends SequentialCommandGroup {

    public PureScore2(CommandSwerveDrivetrain drivetrain) {
        // Defined Start Pose
        Pose2d startPose = new Pose2d(4.004, 7.474, Rotation2d.fromDegrees(0.000));
        
        // Defined Final End Point Target
        Rotation2d endPointHeading = Rotation2d.fromDegrees(-90.000);
        Pose2d targetPose = new Pose2d(1.059, 4.658, endPointHeading);

        // Leg 1: Start -> WP1 -> WP2 -> WP3 -> WP4 (Navigating right and down)
        List<Translation2d> leg1Waypoints = List.of(
            startPose.getTranslation(),      // Start Point
            new Translation2d(6.444, 7.576), // Waypoint 1
            new Translation2d(7.552, 6.872), // Waypoint 2
            new Translation2d(7.853, 5.529), // Waypoint 3
            new Translation2d(7.807, 4.605)  // Waypoint 4
        );
        Rotation2d leg1Heading = Rotation2d.fromDegrees(-75.369); // WP4 Heading

        // Leg 2: WP4 -> WP5 -> WP6 (Looping back left and up)
        List<Translation2d> leg2Waypoints = List.of(
            new Translation2d(7.807, 4.605), // Waypoint 4
            new Translation2d(6.342, 5.530), // Waypoint 5
            new Translation2d(5.836, 7.518)  // Waypoint 6
        );
        Rotation2d leg2Heading = Rotation2d.fromDegrees(110.948); // WP6 Heading

        // Leg 3: WP6 -> WP7 -> WP8 (Traversing across the top left)
        List<Translation2d> leg3Waypoints = List.of(
            new Translation2d(5.836, 7.518), // Waypoint 6
            new Translation2d(4.532, 7.572), // Waypoint 7
            new Translation2d(2.134, 7.513)  // Waypoint 8
        );
        Rotation2d leg3Heading = Rotation2d.fromDegrees(177.212); // WP8 Heading

        // Final leg: WP8 -> End Point (Dipping down to score)
        List<Translation2d> finalWaypoints = List.of(
            new Translation2d(2.134, 7.513), // Waypoint 8
            targetPose.getTranslation()      // End Point
        );

        // Add all phases sequentially to this Command Group
        addCommands(
            // Phase 0: Simulator Start Pose Reset & Ghost Pose Visualization
            drivetrain.runOnce(() -> {
                drivetrain.resetPose(startPose);
                DogLog.log("Auto/TargetPose", targetPose);
            }),

            // Phase 1: Follow Leg 1 to WP4
            drivetrain.getPurePursuitCommand(leg1Waypoints, leg1Heading),

            // Phase 2: Follow Leg 2 to WP6
            drivetrain.getPurePursuitCommand(leg2Waypoints, leg2Heading),

            // Phase 3: Follow Leg 3 across the top to WP8
            drivetrain.getPurePursuitCommand(leg3Waypoints, leg3Heading),

            // Phase 4: Follow the final maneuvering segment from WP8 to the End Point
            drivetrain.getPurePursuitCommand(finalWaypoints, endPointHeading),

            // Phase 5: Final exact alignment using DriveToPose and Vision
            new DriveToPose(drivetrain, targetPose)
        );
    }
}