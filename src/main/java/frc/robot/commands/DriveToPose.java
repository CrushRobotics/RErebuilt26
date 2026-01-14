package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

/**
 * A command that drives the robot to a specific pose on the field.
 * Useful for driving to scoring positions in autonomous.
 */
public class DriveToPose extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;
    
    // PID Controllers for X, Y, and Theta
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    public DriveToPose(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;

        // Max Speed from constants
        double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        double maxAngularRate = Math.PI * 2; // Approx 1 rotation per second

        // PID Profile Constraints (Max Velocity, Max Acceleration)
        var constraints = new TrapezoidProfile.Constraints(
            maxSpeed, 
            2.0 // Max acceleration m/s^2 (Tune this for how snappy you want it)
        );
        var thetaConstraints = new TrapezoidProfile.Constraints(
            maxAngularRate, 
            Math.PI // Max angular acceleration rad/s^2
        );

        // PID Constants (kP, kI, kD) - These are tuned for simulation
        // You might need to adjust these for the real robot.
        xController = new ProfiledPIDController(2.0, 0, 0, constraints);
        yController = new ProfiledPIDController(2.0, 0, 0, constraints);
        thetaController = new ProfiledPIDController(4.0, 0, 0, thetaConstraints);

        // Enable continuous input for rotation (so it takes the shortest path, e.g. 350 -> 10 deg)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Tolerances - When are we "close enough"?
        xController.setTolerance(0.05); // 5 cm
        yController.setTolerance(0.05); // 5 cm
        thetaController.setTolerance(0.05); // ~3 degrees

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset PID controllers to the current robot state to avoid "jumps"
        var currentPose = drivetrain.getState().Pose;
        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());
        thetaController.reset(currentPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        var currentPose = drivetrain.getState().Pose;

        // Calculate desired speeds to reach target
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        // Apply to drivetrain using Field Centric control
        drivetrain.setControl(
            driveRequest.withVelocityX(xSpeed)
                        .withVelocityY(ySpeed)
                        .withRotationalRate(thetaSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        // End the command when we are at the goal within tolerance
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when done
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}