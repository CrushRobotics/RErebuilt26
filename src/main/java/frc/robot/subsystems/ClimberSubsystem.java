package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    // Hardware constraints
    private static final int CLIMBER_MOTOR_ID = 1; 
    
    // The strict hardware limits the motor will not go past (in rotations)
    private static final float MAX_EXTENSION_ROTATIONS = 100.0f; 
    private static final float MIN_EXTENSION_ROTATIONS = 0.0f;

    private final SparkMax climberMotor;
    private final RelativeEncoder climberEncoder;
    private final SparkClosedLoopController climberPid;

    public ClimberSubsystem() {
        // Using the new SparkMax API (Not CANSparkMax)
        climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        
        // --- PID Constants ---
        config.closedLoop.pid(0.1, 0.0, 0.0);
        
        // --- Trapezoidal PID Configuration (REV MAXMotion) ---
        // This generates a smooth trapezoidal curve during movement
        config.closedLoop.maxMotion.maxVelocity(2000); // Maximum speed during travel (RPM)
        config.closedLoop.maxMotion.maxAcceleration(4000); // Speed up/slow down rate (RPM/s)
        config.closedLoop.maxMotion.allowedClosedLoopError(0.5);
        
        // --- Physical Hardware Limits ---
        config.softLimit
            .forwardSoftLimit(MAX_EXTENSION_ROTATIONS)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(MIN_EXTENSION_ROTATIONS)
            .reverseSoftLimitEnabled(true);
        
        // Apply configurations to the controller
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        climberEncoder = climberMotor.getEncoder();
        climberPid = climberMotor.getClosedLoopController();
    }

    /**
     * Drives the climber to a specific position using the Trapezoidal profile.
     * @param targetRotations Position to move to.
     */
    public void setTargetPosition(double targetRotations) {
        // ControlType.kMAXMotionPositionControl utilizes the trapezoidal profile limits
        climberPid.setReference(targetRotations, ControlType.kMAXMotionPositionControl);
        DogLog.log("Climber/TargetRotations", targetRotations);
        SmartDashboard.putNumber("Climber/TargetRotations", targetRotations);
    }
    
    /**
     * Immediately stops the climber motor.
     */
    public void stop() {
        climberMotor.stopMotor();
    }

    /**
     * Gets the current extension value for telemetry.
     */
    public double getExtensionMeters() {
        return climberEncoder.getPosition();
    }

    @Override
    public void periodic() {
        DogLog.log("Climber/CurrentRotations", climberEncoder.getPosition());
        SmartDashboard.putNumber("Climber/CurrentRotations", climberEncoder.getPosition());
    }
}