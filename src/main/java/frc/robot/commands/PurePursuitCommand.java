package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.BallisticSolver.FiringSolution;

import java.util.List;

/**
 * Pure Pursuit Algorithm: Robot chases an imaginary "rabbit" lookahead point on the path.
 * Extremely resilient to being bumped off course.
 */
public class PurePursuitCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final List<Translation2d> path;
    private final Rotation2d desiredHeading;
    private final boolean enableDynamicAiming;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    // TODO: Tune Pure Pursuit lookahead distance. (Larger = smoother but cuts corners; Smaller = tighter but may oscillate)
    private final double lookaheadDistanceMeters = 0.6; 
    private final double completionToleranceMeters = 0.1;

    // TODO: Tune Pure Pursuit PID controllers for your real robot's mass and drivetrain dynamics
    private final PIDController xController = new PIDController(4.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(4.0, 0.0, 0.0);
    
    // Restored ProfiledPIDController for smooth acceleration
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            25.0, 0.0, 0.1,
            new TrapezoidProfile.Constraints(Math.PI * 4, Math.PI * 8)
    );

    public PurePursuitCommand(CommandSwerveDrivetrain drivetrain, List<Translation2d> path, Rotation2d desiredHeading, boolean enableDynamicAiming) {
        this.drivetrain = drivetrain;
        this.path = path;
        this.desiredHeading = desiredHeading;
        this.enableDynamicAiming = enableDynamicAiming;
        addRequirements(drivetrain);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // Resetting the profile to start at our current actual angle
        thetaController.reset(drivetrain.getState().Pose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d robotPos = currentPose.getTranslation();

        Translation2d lookaheadPoint = getLookaheadPoint(robotPos);

        double vx = xController.calculate(robotPos.getX(), lookaheadPoint.getX());
        double vy = yController.calculate(robotPos.getY(), lookaheadPoint.getY());
        
        // --- AIMING LOGIC ---
        // 1. Default to the static path heading passed into the command
        Rotation2d currentTargetHeading = this.desiredHeading;
        
        // 2. Only override with dynamic turret aiming if the flag is true AND we have a physical solution
        if (this.enableDynamicAiming) {
            FiringSolution firingSolution = drivetrain.getCurrentFiringSolution();
            if (firingSolution != null) {
                currentTargetHeading = firingSolution.chassisAimAngle;
            }
        }

        // 3. Calculate rotational rate based on the target heading
        double omega = thetaController.calculate(currentPose.getRotation().getRadians(), currentTargetHeading.getRadians());

        vx = MathUtil.clamp(vx, -3.0, 3.0);
        vy = MathUtil.clamp(vy, -3.0, 3.0);

        drivetrain.setControl(driveRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));

        DogLog.log("PurePursuit/LookaheadPoint", new Pose2d(lookaheadPoint, Rotation2d.kZero));
        // Log the exact angle the solver is demanding so you can verify it in AdvantageScope
        DogLog.log("PurePursuit/TargetHeadingDegrees", currentTargetHeading.getDegrees());
    }

    @Override
    public boolean isFinished() {
        Translation2d finalPoint = path.get(path.size() - 1);
        return drivetrain.getState().Pose.getTranslation().getDistance(finalPoint) < completionToleranceMeters;
    }

    private Translation2d getLookaheadPoint(Translation2d robotPos) {
        Translation2d lookahead = path.get(path.size() - 1);
        for (int i = path.size() - 1; i > 0; i--) {
            Translation2d start = path.get(i - 1);
            Translation2d end = path.get(i);
            Translation2d d = end.minus(start);
            Translation2d f = start.minus(robotPos);

            double a = d.getX() * d.getX() + d.getY() * d.getY();
            double b = 2 * (f.getX() * d.getX() + f.getY() * d.getY());
            double c = (f.getX() * f.getX() + f.getY() * f.getY()) - (lookaheadDistanceMeters * lookaheadDistanceMeters);
            double discriminant = b * b - 4 * a * c;

            if (discriminant >= 0) {
                discriminant = Math.sqrt(discriminant);
                double t = (-b + discriminant) / (2 * a);
                if (t >= 0 && t <= 1) {
                    return start.plus(d.times(t));
                }
            }
        }
        return lookahead;
    }
}