package frc.robot.auto;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonomousLogic {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final CommandSwerveDrivetrain drivetrain;

    public AutonomousLogic(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Register our Trailblazer-style Auto
        autoChooser.setDefaultOption("Trailblazer Style -> Red Hub (Tag 2)", buildTrailblazerAuto());
        
        // Register the PureScore1 Route
        autoChooser.addOption("PureScore1", buildPureScore1());

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Builds an autonomous routine mimicking the Trailblazer architecture.
     * 1. Uses Pure Pursuit to navigate waypoints (the AutoSegments).
     * 2. Uses a final PID servo (DriveToPose) for exact element alignment.
     */
    private Command buildTrailblazerAuto() {
        Pose3d tag3dPose = FieldConstants.APRIL_TAG_FIELD_LAYOUT.get(2);
        if (tag3dPose == null) return drivetrain.runOnce(() -> {}); // Fallback if tag missing
        
        Pose2d tag2dPose = tag3dPose.toPose2d();

        // 1. Calculate our final scoring pose (1 meter in front of the tag)
        Translation2d offset = new Translation2d(1.0, tag2dPose.getRotation());
        Translation2d scoringLocation = tag2dPose.getTranslation().plus(offset);
        Rotation2d scoringHeading = tag2dPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
        Pose2d targetPose = new Pose2d(scoringLocation, scoringHeading);

        Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        // 2. Define our "AutoPoints" (Waypoints)
        // In Trailblazer, these form the "AutoSegments" that the PurePursuitTracker follows.
        List<Translation2d> pathWaypoints = List.of(
            startPose.getTranslation(),                     // Start point
            new Translation2d(1.5, 0.5),                    // Mid-point to dodge a theoretical obstacle
            new Translation2d(scoringLocation.getX() - 0.5, // A point just slightly before the target
                              scoringLocation.getY())
        );

        // 3. Assemble the Command Group
        return new SequentialCommandGroup(
            // Phase 0: Reset Odometry to starting line and show Ghost Target Pose
            drivetrain.runOnce(() -> {
                drivetrain.resetPose(startPose);
                DogLog.log("Auto/TargetPose", targetPose);
            }),

            // Phase 1: Pure Pursuit (The "Tracker" and "Follower")
            // This chases the lookahead point along the segments we defined above.
            drivetrain.getPurePursuitCommand(pathWaypoints, scoringHeading),

            // Phase 2: Exact Vision Alignment
            // Once Pure Pursuit gets us close, DriveToPose acts as the final 
            // tight PID controller utilizing Vision Odometry to lock on.
            new DriveToPose(drivetrain, targetPose)
        );
    }

    /**
     * PureScore1 Autonomous Route
     * Uses strictly user-provided geometric waypoints.
     * Path is split into 3 segments (Outbound, Return, Final) to prevent the lookahead 
     * from jumping segments where the path loops back over its own Y-axis.
     */
    private Command buildPureScore1() {
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

        return new SequentialCommandGroup(
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

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}