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

import java.util.List;

/**
 * Pure Pursuit Algorithm: Robot chases an imaginary "rabbit" lookahead point on the path.
 * Extremely resilient to being bumped off course.
 */
public class PurePursuitCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final List<Translation2d> path;
    private final Rotation2d desiredHeading;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final double lookaheadDistanceMeters = 0.6; 
    private final double completionToleranceMeters = 0.1;

    private final PIDController xController = new PIDController(4.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(4.0, 0.0, 0.0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            4.0, 0.0, 0.1,
            new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 4)
    );

    public PurePursuitCommand(CommandSwerveDrivetrain drivetrain, List<Translation2d> path, Rotation2d desiredHeading) {
        this.drivetrain = drivetrain;
        this.path = path;
        this.desiredHeading = desiredHeading;
        addRequirements(drivetrain);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
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
        double omega = thetaController.calculate(currentPose.getRotation().getRadians(), desiredHeading.getRadians());

        vx = MathUtil.clamp(vx, -3.0, 3.0);
        vy = MathUtil.clamp(vy, -3.0, 3.0);

        drivetrain.setControl(driveRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));

        DogLog.log("PurePursuit/LookaheadPoint", new Pose2d(lookaheadPoint, Rotation2d.kZero));
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