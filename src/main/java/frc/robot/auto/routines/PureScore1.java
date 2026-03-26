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

public class PureScore1 extends SequentialCommandGroup {

    public PureScore1(CommandSwerveDrivetrain drivetrain, HoodSubsystem hood, ShooterSubsystem shooter, IndexerSubsystem indexer) {
        Pose2d startPose = new Pose2d(4.047, 0.629, Rotation2d.fromDegrees(-5.540));
        
        // --- PHASE 1: Drive to Fuel ---
        List<Translation2d> phase1Waypoints = List.of(
            startPose.getTranslation(),      
            new Translation2d(6.205, 0.641)  // WP1 (Approaching Centerline)
        );
        Rotation2d phase1Heading = Rotation2d.fromDegrees(0.0);

        // --- PHASE 2: Intake/Sweep ---
        List<Translation2d> phase2Waypoints = List.of(
            new Translation2d(6.205, 0.641), // WP1
            new Translation2d(7.786, 0.888), // WP2 (On Centerline)
            new Translation2d(7.773, 3.657)  // WP3 (Sweeping up)
        );
        // Fixed intake heading between 80-100 degrees
        Rotation2d phase2Heading = Rotation2d.fromDegrees(90.0); 

        // --- PHASE 3: Drive Back (Crossing Trench) ---
        List<Translation2d> phase3Waypoints = List.of(
            new Translation2d(7.773, 3.657), // WP3
            new Translation2d(6.852, 3.105), // WP4
            new Translation2d(6.911, 0.654)  // WP5 (Crossed Trench)
        );
        // Base heading set to 0.0, keeping orientation neutral while crossing back over.
        Rotation2d phase3Heading = Rotation2d.fromDegrees(0.0);

        // --- PHASE 4: Drive to Depot (Shooter Phase Begins!) ---
        List<Translation2d> phase4Waypoints = List.of(
            new Translation2d(6.911, 0.654), // WP5
            new Translation2d(3.040, 0.614), // WP6
            new Translation2d(1.018, 0.614)  // WP7 (Depot at the back wall)
        );
        // Base heading is 0.0, but dynamic aiming will override this to keep it locked on the Hub
        Rotation2d depotHeading = Rotation2d.fromDegrees(0.0);
        
        // --- PHASE 5: Climb Approach ---
        // Face 0.0 degrees. This keeps the climber (side) perpendicular to the rung for a clean strafe
        Rotation2d endPointHeading = Rotation2d.fromDegrees(0.0);
        
        // Target Pose moves strictly in Y from WP7 to align with the rung while strafing
        Pose2d targetPose = new Pose2d(1.018, 2.815, endPointHeading);

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

            // Phase 3: Cross trench back to our side (No dynamic aiming yet)
            drivetrain.getPurePursuitCommand(phase3Waypoints, phase3Heading, false),

            // Phase 4, Pause, and Phase 5: Run parallel with the shooting logic
            Commands.parallel(
                Commands.sequence(
                    // Phase 4: Drive to Depot. Dynamic Aiming TRUE to lock onto Hub!
                    drivetrain.getPurePursuitCommand(phase4Waypoints, depotHeading, true),
                    
                    // Pause for 2.5 seconds at the Depot.
                    // The drivetrain holds its locked heading while the human player feeds balls.
                    Commands.waitSeconds(2.5),
                    
                    // Phase 5: Final strafe to the climber rung
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