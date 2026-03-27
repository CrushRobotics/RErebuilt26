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

/**
 * A testing routine that strips out all weapons/combat systems.
 * It ONLY runs the path and the dynamic Hub alignment to verify the drivetrain math.
 */
public class PureScore1PATH extends SequentialCommandGroup {

    public PureScore1PATH(CommandSwerveDrivetrain drivetrain) {
        // The exact starting physical location on the carpet
        Pose2d startPose = new Pose2d(4.047, 0.629, Rotation2d.fromDegrees(-5.540));
        
        // --- PHASE 1: Drive to Fuel ---
        List<Translation2d> phase1Waypoints = List.of(
            startPose.getTranslation(),      
            new Translation2d(6.205, 0.641)
        );
        Rotation2d phase1Heading = Rotation2d.fromDegrees(0.0);

        // --- PHASE 2: Intake/Sweep ---
        List<Translation2d> phase2Waypoints = List.of(
            new Translation2d(6.205, 0.641),
            new Translation2d(7.786, 0.888),
            new Translation2d(7.773, 3.657)
        );
        Rotation2d phase2Heading = Rotation2d.fromDegrees(90.0); 

        // --- PHASE 3: Drive Back (Crossing Trench) ---
        List<Translation2d> phase3Waypoints = List.of(
            new Translation2d(7.773, 3.657),
            new Translation2d(6.852, 3.105),
            new Translation2d(6.911, 0.654)
        );
        Rotation2d phase3Heading = Rotation2d.fromDegrees(180.0);

        // --- PHASE 4: Drive to Depot ---
        List<Translation2d> phase4Waypoints = List.of(
            new Translation2d(6.911, 0.654),
            new Translation2d(3.040, 0.614),
            new Translation2d(1.018, 0.614)
        );
        
        // --- PHASE 5: Climb Approach ---
        List<Translation2d> phase5Waypoints = List.of(
            new Translation2d(1.018, 0.614),
            new Translation2d(1.018, 2.815)
        );

        Rotation2d dummyHeading = Rotation2d.kZero;

        // Persistent aim hold request to prevent PID jitter
        SwerveRequest.FieldCentricFacingAngle aimHoldRequest = new SwerveRequest.FieldCentricFacingAngle();
        aimHoldRequest.HeadingController.setPID(8.0, 0, 0.5); 
        aimHoldRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addCommands(
            // 1. ESTABLISH POSITION: Overrides the robot's odometry to the exact start pose
            drivetrain.runOnce(() -> {
                drivetrain.resetPose(startPose);
                DogLog.log("Auto/TargetPose", new Pose2d(1.018, 2.815, Rotation2d.kZero));
            }),

            // 2. Execute Paths (No Aiming)
            drivetrain.getPurePursuitCommand(phase1Waypoints, phase1Heading, false),
            drivetrain.getPurePursuitCommand(phase2Waypoints, phase2Heading, false),
            drivetrain.getPurePursuitCommand(phase3Waypoints, phase3Heading, false),

            // 3. Execute Paths (DYNAMIC AIMING ENABLED)
            drivetrain.getPurePursuitCommand(phase4Waypoints, dummyHeading, true),
            
            // 4. Depot Pause (Active Aiming)
            Commands.run(() -> {
                Rotation2d aimAngle = dummyHeading;
                if (drivetrain.getCurrentFiringSolution() != null) {
                    aimAngle = drivetrain.getCurrentFiringSolution().chassisAimAngle;
                }
                drivetrain.setControl(
                    aimHoldRequest.withVelocityX(0).withVelocityY(0).withTargetDirection(aimAngle)
                );
            }, drivetrain).withTimeout(2.5),
            
            // 5. Final Strafe (Dynamic Aiming)
            drivetrain.getPurePursuitCommand(phase5Waypoints, dummyHeading, true),

            // 6. Stop Motors
            Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.SwerveDriveBrake()))
        );
    }
}