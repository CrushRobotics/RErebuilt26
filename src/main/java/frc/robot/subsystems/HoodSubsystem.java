package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    // Hardware constraints
    private static final int HOOD_MOTOR_ID = 14; 
    
    private final SparkMax hoodMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController hoodPid;
    
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0.0);
    
   
    private static final double GEAR_RATIO = 420.0; 
    
    private static final double POSITION_TOLERANCE_DEGREES = 2.0;

    public HoodSubsystem() {
        hoodMotor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Brake mode helps fight gravity
        config.idleMode(IdleMode.kBrake);

        // Convert NEO rotations directly to Hood Degrees using your gear ratio
        config.encoder.positionConversionFactor(360.0 / GEAR_RATIO); 
        config.encoder.velocityConversionFactor((360.0 / GEAR_RATIO) / 60.0);
        
        // --- PID Constants ---
        // Hardware PID values are usually much smaller than WPILib PID values!
        // Start this at 0.01 and slowly increase it until it tracks well.
        config.closedLoop.pid(0.02, 0.0, 0.0);
        
        // --- Trapezoidal PID Configuration (REV MAXMotion) ---
        // This generates the buttery smooth profile at 1000Hz on the hardware
        config.closedLoop.maxMotion.maxVelocity(180); // Max speed (Degrees per second)
        config.closedLoop.maxMotion.maxAcceleration(360); // Max accel (Degrees per second squared)
        config.closedLoop.maxMotion.allowedClosedLoopError(POSITION_TOLERANCE_DEGREES);
        
        // Apply configuration to the SparkMax
        hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = hoodMotor.getEncoder();
        hoodPid = hoodMotor.getClosedLoopController();
        
        // Reset encoder on boot (Assumes hood starts at a known 0 position, like hard-stopped)
        encoder.setPosition(0);
    }

    public void setTargetAngle(Rotation2d angle) {
        targetAngle = angle;
    }

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
        // Feed the target to the Spark Max's internal 1000Hz loop
        hoodPid.setReference(targetAngle.getDegrees(), ControlType.kMAXMotionPositionControl);

        // Logging
        DogLog.log("Hood/TargetAngleDeg", targetAngle.getDegrees());
        DogLog.log("Hood/CurrentAngleDeg", getCurrentAngleDegrees());
        DogLog.log("Hood/AppliedOutput", hoodMotor.getAppliedOutput());
        DogLog.log("Hood/AtTarget", isAtAngle(targetAngle));
    }
}