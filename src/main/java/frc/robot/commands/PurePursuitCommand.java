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

public class PurePursuitCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final List<Translation2d> path;
    private final Rotation2d desiredHeading;
    private final boolean enableDynamicAiming;

    // Normal request for sweeping/driving
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    // Native CTRE Request for aggressive Auto-Aiming on the move!
    private final SwerveRequest.FieldCentricFacingAngle autoAimDriveRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final double lookaheadDistanceMeters = 0.6; 
    private final double completionToleranceMeters = 0.1;

    private final PIDController xController = new PIDController(4.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(4.0, 0.0, 0.0);
    
    // Only used when dynamic aiming is FALSE
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            8.0, 0.0, 0.1,
            new TrapezoidProfile.Constraints(Math.PI * 4, Math.PI * 8)
    );

    public PurePursuitCommand(CommandSwerveDrivetrain drivetrain, List<Translation2d> path, Rotation2d desiredHeading, boolean enableDynamicAiming) {
        this.drivetrain = drivetrain;
        this.path = path;
        this.desiredHeading = desiredHeading;
        this.enableDynamicAiming = enableDynamicAiming;
        addRequirements(drivetrain);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Apply the same aggressive PID we found worked in teleop
        autoAimDriveRequest.HeadingController.setPID(20.0, 0, 1.0);
        autoAimDriveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        thetaController.reset(drivetrain.getState().Pose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d robotPos = currentPose.getTranslation();
        Translation2d lookaheadPoint = getLookaheadPoint(robotPos);

        double vx = xController.calculate(robotPos.getX(), lookaheadPoint.getX());
        double vy = yController.calculate(robotPos.getY(), lookaheadPoint.getY());
        
        vx = MathUtil.clamp(vx, -3.0, 3.0);
        vy = MathUtil.clamp(vy, -3.0, 3.0);

        // --- DYNAMIC AIMING LOGIC (LIKE A TURRET) ---
        if (this.enableDynamicAiming) {
            FiringSolution firingSolution = drivetrain.getCurrentFiringSolution();
            if (firingSolution != null) {
                // Hand the math directly to CTRE's internal closed-loop to eliminate lag
                drivetrain.setControl(autoAimDriveRequest
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withTargetDirection(firingSolution.chassisAimAngle)
                );
                DogLog.log("PurePursuit/TargetHeadingDegrees", firingSolution.chassisAimAngle.getDegrees());
                return; // Exit early!
            }
        }

        // --- STATIC AIMING LOGIC ---
        double omega = thetaController.calculate(currentPose.getRotation().getRadians(), desiredHeading.getRadians());
        drivetrain.setControl(driveRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
        DogLog.log("PurePursuit/TargetHeadingDegrees", desiredHeading.getDegrees());
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