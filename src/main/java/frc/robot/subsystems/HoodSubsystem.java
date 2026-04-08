package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    // Note: Update the CAN ID (e.g., 15) to match your physical robot configuration
    private final SparkMax hoodMotor = new SparkMax(14, MotorType.kBrushless);
    private final RelativeEncoder encoder = hoodMotor.getEncoder();

    // Trapezoidal Profiled PID Controller for buttery smooth hood movement
    private final ProfiledPIDController profiledPID;
    
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0.0);
    
    // Note: Update this gear ratio to match your actual hood mechanism
    private static final double GEAR_RATIO = 1.0; 
    private static final double POSITION_TOLERANCE_DEGREES = 2.0;

    public HoodSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Convert NEO rotations to Hood Degrees
        config.encoder.positionConversionFactor(360.0 / GEAR_RATIO); 
        config.encoder.velocityConversionFactor((360.0 / GEAR_RATIO) / 60.0);
        
        // Apply configuration to the SparkMax
        hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure the Trapezoidal Profile
        // Parameters: P, I, D, Constraints(Max Velocity, Max Acceleration)
        profiledPID = new ProfiledPIDController(
            0.1, 0.0, 0.0, // Bumped P from 0.05 to 0.1 to give it a little more muscle to overcome friction
            new TrapezoidProfile.Constraints(
                180.0, // Max velocity (Degrees per second)
                360.0  // Max acceleration (Degrees per second squared)
            )
        );
        profiledPID.setTolerance(POSITION_TOLERANCE_DEGREES);
        
        // Reset encoder on boot (Assumes hood starts at a known 0 position, like hard-stopped)
        encoder.setPosition(0);
    }

    public void setTargetAngle(Rotation2d angle) {
        // The ProfiledPIDController handles dynamic setpoint changes natively.
        // Do NOT reset it here, or it will stutter during auto-aim tracking.
        targetAngle = angle;
    }

    // Added this getter so we can increment off the TARGET, not the current position
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public boolean isAtAngle(Rotation2d angle) {
        return Math.abs(getCurrentAngleDegrees() - angle.getDegrees()) <= POSITION_TOLERANCE_DEGREES;
    }

    public double getCurrentAngleDegrees() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        // Calculate the profiled motor output (in Volts) based on the trapezoidal motion plan
        double pidOutputVolts = profiledPID.calculate(getCurrentAngleDegrees(), targetAngle.getDegrees());
        
        // Apply voltage to the NEO
        hoodMotor.setVoltage(pidOutputVolts);

        
        DogLog.log("Hood/TargetAngleDeg", targetAngle.getDegrees());
        DogLog.log("Hood/CurrentAngleDeg", getCurrentAngleDegrees());
        DogLog.log("Hood/AppliedVolts", pidOutputVolts);
        DogLog.log("Hood/ProfiledSetpointDeg", profiledPID.getSetpoint().position);
        DogLog.log("Hood/AtTarget", isAtAngle(targetAngle));
    }
}