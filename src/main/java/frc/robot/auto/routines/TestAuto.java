package frc.robot.auto.routines;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto(CommandSwerveDrivetrain drivetrain, HoodSubsystem hood, ShooterSubsystem shooter, IndexerSubsystem indexer) {
        // Starts exactly where PureScore1 starts
        Pose2d startPose = new Pose2d(4.047, 0.629, Rotation2d.fromDegrees(-5.540));
        
        // --- PHASE 1: Drive Straight for 2 Meters ---
        List<Translation2d> phase1Waypoints = List.of(
            startPose.getTranslation(),      
            new Translation2d(6.047, 0.629)  // +2 meters in the X direction
        );
        // Face forward
        Rotation2d phase1Heading = Rotation2d.fromDegrees(0.0);

        // --- PHASE 2: Rotate to Intake Side & Drive Another 2 Meters ---
        List<Translation2d> phase2Waypoints = List.of(
            new Translation2d(6.047, 0.629),
            new Translation2d(8.047, 0.629)  // +2 more meters in the X direction
        );
        // Rotate 180 degrees (Assuming intake is facing backward relative to the original path).
        // Change to 90.0 if you want to test strafing while facing the trench!
        Rotation2d phase2Heading = Rotation2d.fromDegrees(180.0); 

        addCommands(
            drivetrain.runOnce(() -> {
                drivetrain.resetPose(startPose);
                DogLog.log("Auto/TargetPose", new Pose2d(8.047, 0.629, phase2Heading));
            }),

            // Phase 1: Go straight 2 meters
            drivetrain.getPurePursuitCommand(phase1Waypoints, phase1Heading, false),

            // Phase 2: Rotate and go another 2 meters
            drivetrain.getPurePursuitCommand(phase2Waypoints, phase2Heading, false),

            // Cleanup: Stop the motors so it doesn't spin out
            Commands.runOnce(() -> {
                drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
            })
        );
    }
}