package frc.robot.auto.routines;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

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
        Rotation2d phase2Heading = Rotation2d.fromDegrees(90.0); 

        // --- PHASE 3: Drive Back (Crossing Trench) ---
        List<Translation2d> phase3Waypoints = List.of(
            new Translation2d(7.773, 3.657), // WP3
            new Translation2d(6.852, 3.105), // WP4
            new Translation2d(6.911, 0.654)  // WP5 (Crossed Trench)
        );
        // SMOOTH ALIGNMENT: Point to our side (180 degrees) instead of jittery dynamic aiming.
        Rotation2d phase3Heading = Rotation2d.fromDegrees(180.0);

        // --- PHASE 4: Drive to Depot ---
        List<Translation2d> phase4Waypoints = List.of(
            new Translation2d(6.911, 0.654), // WP5
            new Translation2d(3.040, 0.614), // WP6
            new Translation2d(1.018, 0.614)  // WP7 (Depot at the back wall)
        );
        
        // --- PHASE 5: Climb Approach ---
        List<Translation2d> phase5Waypoints = List.of(
            new Translation2d(1.018, 0.614), // WP7 (Depot)
            new Translation2d(1.018, 2.815)  // WP8 (Climber Rung)
        );

        // A dummy heading. When enableDynamicAiming is TRUE, this is completely ignored.
        Rotation2d dummyHeading = Rotation2d.kZero;

        // FIX: Create a single, tuned request so the PID controller remembers its state and damps the oscillation
        SwerveRequest.FieldCentricFacingAngle aimHoldRequest = new SwerveRequest.FieldCentricFacingAngle();
        // Lower P to stop violent overshooting, add D to act as a brake as it reaches the angle
        aimHoldRequest.HeadingController.setPID(8.0, 0, 0.5); 
        aimHoldRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addCommands(
            drivetrain.runOnce(() -> {
                drivetrain.resetPose(startPose);
                DogLog.log("Auto/TargetPose", new Pose2d(1.018, 2.815, Rotation2d.kZero));
            }),

            // Phase 1: Drive to Fuel (No dynamic aiming)
            drivetrain.getPurePursuitCommand(phase1Waypoints, phase1Heading, false),

            // Phase 2: Sweep Fuel at 90 Degrees
            drivetrain.getPurePursuitCommand(phase2Waypoints, phase2Heading, false),

            // --- PHASE 3: "Shoot to our side" ---
            Commands.deadline(
                drivetrain.getPurePursuitCommand(phase3Waypoints, phase3Heading, false),
                Commands.run(() -> {
                    // Fixed lob to our side of the field
                    hood.setTargetAngle(Rotation2d.fromDegrees(40.0));
                    shooter.setTargetVelocity(12.0); 

                    // We don't check chassis alignment since we are just lobbing it to our side
                    if (shooter.isAtVelocity(12.0) && hood.isAtAngle(Rotation2d.fromDegrees(40.0))) {
                        indexer.feedAllBalls();
                    } else {
                        indexer.stopFeeder();
                    }
                }, hood, shooter, indexer)
            ),

            // --- PHASES 4 and 5: Precision Depot Approach ---
            // Changed from parallel to deadline so it actually finishes when the path is done!
            Commands.deadline(
                Commands.sequence(
                    // Phase 4: Drive to Depot (Dynamic aiming turns on here for precision)
                    drivetrain.getPurePursuitCommand(phase4Waypoints, dummyHeading, true),
                    
                    // ACTIVE Aim Hold at the depot for 2.5 seconds to catch human player balls
                    Commands.run(() -> {
                        Rotation2d aimAngle = dummyHeading;
                        if (drivetrain.getCurrentFiringSolution() != null) {
                            aimAngle = drivetrain.getCurrentFiringSolution().chassisAimAngle;
                        }
                        
                        // Use our persistent, smoothed request instead of a 'new' raw one
                        drivetrain.setControl(
                            aimHoldRequest
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withTargetDirection(aimAngle)
                        );
                    }, drivetrain).withTimeout(2.5),
                    
                    // Final strafe to the climber rung
                    drivetrain.getPurePursuitCommand(phase5Waypoints, dummyHeading, true)
                ), 
                
                // This gate command runs continuously during Phases 4 and 5.
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
            // Without this, the swerve modules get stuck on their last target angle and spin haywire.
            Commands.runOnce(() -> {
                drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
                shooter.stop();
                indexer.stopFeeder();
            })
        );
    }
}