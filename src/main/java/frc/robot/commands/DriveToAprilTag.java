package frc.robot.commands;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.*;

public class DriveToAprilTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final int tagId;
    
    // Trajectory objects
    private Trajectory trajectory;
    private final Timer timer = new Timer();
    
    // Controllers for following
    private final HolonomicDriveController controller;
    // Use FieldCentric request since ApplyChassisSpeeds might not be available
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    // Configuration
    private static final double kMaxSpeedMetersPerSecond = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    private static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    private static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    // PIDs (Tune these!)
    private static final double kPXController = 1.0;
    private static final double kPYController = 1.0;
    private static final double kPThetaController = 1.0;

    public DriveToAprilTag(CommandSwerveDrivetrain drivetrain, int tagId) {
        this.drivetrain = drivetrain;
        this.tagId = tagId;

        // Setup PID controllers
        PIDController xController = new PIDController(kPXController, 0, 0);
        PIDController yController = new PIDController(kPYController, 0, 0);
        
        ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, 0, 0,
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared)
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.controller = new HolonomicDriveController(xController, yController, thetaController);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Pose3d targetTagPose = FieldConstants.APRIL_TAG_FIELD_LAYOUT.get(tagId);

        if (targetTagPose == null) {
            System.out.println("Error: Tag ID " + tagId + " not found in FieldConstants!");
            end(true);
            return;
        }

        // Convert Tag 3D pose to 2D target.
        // Usually you want to stop *in front* of the tag, but for this example
        // we will drive to the exact tag location 1 meter in front of it to avoid crashing.
        // The tag rotation is facing OUT from the wall. We want to face the wall (opposite tag rotation).
        Pose2d targetPose = targetTagPose.toPose2d();
        
        // Offset target by 1 meter in front of the tag so we don't hit the wall.
        // Tag rotation usually points OUT of the wall. 
        // So moving +1.0 meter in the direction of the tag's rotation puts us 1m away.
        Translation2d offset = new Translation2d(1.0, targetPose.getRotation());
        Pose2d driveTarget = new Pose2d(
            targetPose.getTranslation().plus(offset), 
            targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)) // Face the tag
        );

        // Config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond * 0.5, // Run at half speed for safety
            kMaxAccelerationMetersPerSecondSquared
        )
        // Ensure we start at our current speed
        .setStartVelocity(0)
        .setEndVelocity(0);

        // Generate the path
        try {
            trajectory = TrajectoryGenerator.generateTrajectory(
                currentPose,
                List.of(), // No interior waypoints, just straight path finding
                driveTarget,
                config
            );
            
            DogLog.log("Auto/Trajectory/Generated", true);
            DogLog.log("Auto/Trajectory/TargetPose", driveTarget);
        } catch (Exception e) {
            System.out.println("Failed to generate trajectory: " + e.getMessage());
            end(true);
            return;
        }

        timer.restart();
    }

    @Override
    public void execute() {
        if (trajectory == null) return;

        double curTime = timer.get();
        Trajectory.State desiredState = trajectory.sample(curTime);

        // Log where the trajectory thinks we should be
        DogLog.log("Auto/Trajectory/DesiredPose", desiredState.poseMeters);

        // Calculate chassis speeds
        ChassisSpeeds targetSpeeds = controller.calculate(
            drivetrain.getState().Pose,
            desiredState,
            desiredState.poseMeters.getRotation()
        );

        // Apply to swerve
        drivetrain.setControl(
            driveRequest.withVelocityX(targetSpeeds.vxMetersPerSecond)
                        .withVelocityY(targetSpeeds.vyMetersPerSecond)
                        .withRotationalRate(targetSpeeds.omegaRadiansPerSecond)
        );
    }

    @Override
    public boolean isFinished() {
        if (trajectory == null) return true;
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}