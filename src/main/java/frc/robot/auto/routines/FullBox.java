package frc.robot.auto.routines;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FullBox extends SequentialCommandGroup {

    public FullBox(CommandSwerveDrivetrain drivetrain) {
        // Defined Start Pose
        Pose2d startPose = new Pose2d(4.013, 7.255, Rotation2d.fromDegrees(171.935));
        
        // Defined Final End Point Target
        Rotation2d endPointHeading = Rotation2d.fromDegrees(-90.843);
        Pose2d targetPose = new Pose2d(1.090, 4.617, endPointHeading);

        // Leg 1: Start -> WP1 to WP7 (Traversing left, down, and across to the right)
        List<Translation2d> leg1Waypoints = List.of(
            startPose.getTranslation(),      // Start Point
            new Translation2d(0.832, 7.710), // Waypoint 1
            new Translation2d(0.372, 5.845), // Waypoint 2
            new Translation2d(0.378, 4.821), // Waypoint 3
            new Translation2d(2.356, 4.930), // Waypoint 4
            new Translation2d(5.067, 5.079), // Waypoint 5
            new Translation2d(6.610, 5.111), // Waypoint 6
            new Translation2d(7.849, 5.110)  // Waypoint 7
        );
        Rotation2d leg1Heading = Rotation2d.fromDegrees(34.648); // WP7 Heading

        // Leg 2: WP7 -> WP8 to WP10 (Traversing up and back across the top left)
        List<Translation2d> leg2Waypoints = List.of(
            new Translation2d(7.849, 5.110), // Waypoint 7
            new Translation2d(7.955, 6.741), // Waypoint 8
            new Translation2d(6.240, 7.420), // Waypoint 9
            new Translation2d(4.636, 7.372)  // Waypoint 10
        );
        Rotation2d leg2Heading = Rotation2d.fromDegrees(-177.760); // WP10 Heading

        // Final leg: WP10 -> End Point (Closing the box)
        List<Translation2d> finalWaypoints = List.of(
            new Translation2d(4.636, 7.372), // Waypoint 10
            targetPose.getTranslation()      // End Point
        );

        // Add all phases sequentially to this Command Group
        addCommands(
            // Phase 0: Simulator Start Pose Reset & Ghost Pose Visualization
            drivetrain.runOnce(() -> {
                drivetrain.resetPose(startPose);
                DogLog.log("Auto/TargetPose", targetPose);
            }),

            // Phase 1: Follow Leg 1 across the bottom/middle to WP7
            drivetrain.getPurePursuitCommand(leg1Waypoints, leg1Heading),

            // Phase 2: Follow Leg 2 across the top to WP10
            drivetrain.getPurePursuitCommand(leg2Waypoints, leg2Heading),

            // Phase 3: Follow the final maneuvering segment from WP10 to the End Point
            drivetrain.getPurePursuitCommand(finalWaypoints, endPointHeading),

            // Phase 4: Final exact alignment using DriveToPose and Vision
            new DriveToPose(drivetrain, targetPose)
        );
    }
}