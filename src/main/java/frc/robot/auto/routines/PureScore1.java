package frc.robot.auto.routines;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PureScore1 extends SequentialCommandGroup {

    public PureScore1(CommandSwerveDrivetrain drivetrain) {
        // Defined Start Pose
        Pose2d startPose = new Pose2d(4.047, 0.629, Rotation2d.fromDegrees(-5.540));
        
        // Defined Final End Point Target
        Rotation2d endPointHeading = Rotation2d.fromDegrees(90.0);
        Pose2d targetPose = new Pose2d(1.018, 2.815, endPointHeading);

        // Outbound leg: Start -> WP1 -> WP2 -> WP3 (Turnaround in the fuel zone)
        List<Translation2d> outboundWaypoints = List.of(
            startPose.getTranslation(),      // Start Point
            new Translation2d(6.205, 0.641), // Waypoint 1
            new Translation2d(7.786, 0.888), // Waypoint 2
            new Translation2d(7.773, 3.657)  // Waypoint 3: Turnaround
        );
        Rotation2d outboundHeading = Rotation2d.fromDegrees(160.260); // WP3 Heading

        // Return leg: WP3 -> WP4 -> WP5 -> WP6 -> WP7 (Return to scoring area)
        List<Translation2d> returnWaypoints = List.of(
            new Translation2d(7.773, 3.657), // Waypoint 3: Start Return
            new Translation2d(6.852, 3.105), // Waypoint 4
            new Translation2d(6.911, 0.654), // Waypoint 5
            new Translation2d(3.040, 0.614), // Waypoint 6
            new Translation2d(0.272, 0.675)  // Waypoint 7
        );
        Rotation2d returnHeading = Rotation2d.fromDegrees(90.112); // WP7 Heading

        // Final leg: WP7 -> End Point (Handles the backup/re-align)
        List<Translation2d> finalWaypoints = List.of(
            new Translation2d(0.272, 0.675), // Waypoint 7
            targetPose.getTranslation()      // End Point
        );

        // Add all phases sequentially to this Command Group
        addCommands(
            // Phase 0: Simulator Start Pose Reset & Ghost Pose Visualization
            drivetrain.runOnce(() -> {
                drivetrain.resetPose(startPose);
                DogLog.log("Auto/TargetPose", targetPose);
            }),

            // Phase 1: Follow the outbound leg to WP3
            drivetrain.getPurePursuitCommand(outboundWaypoints, outboundHeading),

            // Phase 2: Follow the return leg to WP7
            drivetrain.getPurePursuitCommand(returnWaypoints, returnHeading),

            // Phase 3: Follow the final maneuvering segment from WP7 to the End Point
            drivetrain.getPurePursuitCommand(finalWaypoints, endPointHeading),

            // Phase 4: Final exact alignment using DriveToPose and Vision
            new DriveToPose(drivetrain, targetPose)
        );
    }
}