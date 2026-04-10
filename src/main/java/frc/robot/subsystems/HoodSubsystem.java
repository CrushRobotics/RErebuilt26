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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    private static final int HOOD_MOTOR_ID = 14; 
    private final SparkMax hoodMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController hoodPid;
    
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0.0);
    private boolean isManualStop = false; // Flag to prevent PID from overriding the stop command
    
    // High reduction gearbox (Typical for hoods)
    private static final double GEAR_RATIO = 420.0; 
    
    // Lowered tolerance so small auto-aim adjustments aren't ignored
    private static final double POSITION_TOLERANCE_DEGREES = 0.5;

    public HoodSubsystem() {
        hoodMotor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Brake mode is essential to stop the hood from falling under its own weight
        config.idleMode(IdleMode.kBrake);
        
        // Invert the motor direction so 'up' goes the correct way
        config.inverted(true);

        // Convert NEO rotations directly to Hood Degrees
        config.encoder.positionConversionFactor(360.0 / GEAR_RATIO); 
        config.encoder.velocityConversionFactor((360.0 / GEAR_RATIO) / 60.0);
        
        // --- PID Constants ---
        // Increased kP significantly from 0.1 to 0.5 to make it snap to position much faster
        config.closedLoop.pid(2.0, 0.0, 0.0);
        
        
        config.closedLoop.maxMotion.maxAcceleration(1000); // Degrees per second squared (up from 360)
        config.closedLoop.maxMotion.allowedClosedLoopError(POSITION_TOLERANCE_DEGREES);
        
        // Apply configuration to the SparkMax
        hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = hoodMotor.getEncoder();
        hoodPid = hoodMotor.getClosedLoopController();
        
        // Reset encoder on boot
        encoder.setPosition(0);
    }

    public void setTargetAngle(Rotation2d angle) {
        isManualStop = false; // Reset the manual stop flag when a new target is commanded
        targetAngle = angle;
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public boolean isAtAngle(Rotation2d angle) {
        return Math.abs(getCurrentAngleDegrees() - angle.getDegrees()) <= (POSITION_TOLERANCE_DEGREES * 2.0);
    }

    public double getCurrentAngleDegrees() {
        return encoder.getPosition();
    }

    /** * Immediately cuts power to the hood motor and flags the PID to stop updating. 
     */
    public void stop() {
        isManualStop = true;
        hoodMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // Only feed the target to the Spark Max if the manual stop hasn't been triggered
        if (!isManualStop) {
            hoodPid.setReference(targetAngle.getDegrees(), ControlType.kMAXMotionPositionControl);
        }

        // Telemetry
        DogLog.log("Hood/TargetAngleDeg", targetAngle.getDegrees());
        DogLog.log("Hood/CurrentAngleDeg", getCurrentAngleDegrees());
        DogLog.log("Hood/AtTarget", isAtAngle(targetAngle));
        DogLog.log("Hood/IsManualStopped", isManualStop);

        // SmartDashboard debugging
        SmartDashboard.putNumber("Hood/TargetAngle", targetAngle.getDegrees());
        SmartDashboard.putNumber("Hood/CurrentAngle", getCurrentAngleDegrees());
        SmartDashboard.putNumber("Hood/Output", hoodMotor.getAppliedOutput());
    }
}