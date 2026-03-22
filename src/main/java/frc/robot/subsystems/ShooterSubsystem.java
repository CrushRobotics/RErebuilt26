package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.DogLog;

public class ShooterSubsystem extends SubsystemBase {
    // TODO: Verify your CAN IDs
    private static final int LEFT_KRAKEN_ID = 20;
    private static final int RIGHT_KRAKEN_ID = 21;

    // IMPORTANT: Verify the physical diameter of your shooter wheels (e.g., 4 inches)
    private static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    
    // IMPORTANT: If your Kraken isn't 1:1 direct drive with the wheel, set the ratio here
    private static final double GEAR_RATIO = 1.0; 

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    // CTRE Phoenix 6 Velocity control request
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    
    private double currentTargetRpm = 0.0;

    public ShooterSubsystem() {
        leftMotor = new TalonFX(LEFT_KRAKEN_ID);
        rightMotor = new TalonFX(RIGHT_KRAKEN_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // TODO: Tune Kraken PID for your specific flywheels
        config.Slot0.kP = 0.11; 
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12; // Velocity Feedforward is critical for fast recovery!

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        // Run the right motor as a strict follower of the left (inverted so they spin the ball out)
        // Phoenix 6 Follower (2025/2026+) takes the MotorAlignmentValue in the constructor
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setTargetVelocity(double velocityMps) {
        double targetRpm = convertMpsToRpm(velocityMps);
        this.currentTargetRpm = targetRpm;

        // Phoenix 6 expects Rotations Per Second (RPS)
        double rotationsPerSecond = targetRpm / 60.0;
        leftMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    public double getCurrentTargetRpm() {
        return currentTargetRpm;
    }

    public boolean isAtVelocity(double targetVelocityMps) {
        double targetRpm = convertMpsToRpm(targetVelocityMps);
        
        // Phoenix 6 reports velocity in RPS, convert to RPM
        double currentRpm = leftMotor.getVelocity().getValueAsDouble() * 60.0;
        
        // Smart Gate Check: Are we within 100 RPM of the exact ballistic target?
        return Math.abs(currentRpm - targetRpm) < 100.0; 
    }

    public void stop() {
        leftMotor.setControl(new com.ctre.phoenix6.controls.DutyCycleOut(0));
        currentTargetRpm = 0;
    }

    /**
     * Converts linear exit velocity (m/s) to Flywheel RPM using the circumference of the wheel.
     */
    private double convertMpsToRpm(double metersPerSecond) {
        // Linear Velocity = RPM * Circumference / 60
        // Therefore: RPM = (Velocity * 60) / (PI * Diameter)
        double wheelRPM = (metersPerSecond * 60.0) / (Math.PI * WHEEL_DIAMETER_METERS);
        
        // Multiply by gear ratio in case the Krakens are geared up/down relative to the wheel
        return wheelRPM * GEAR_RATIO;
    }

    @Override
    public void periodic() {
        // Log both the current actual RPM and the current Target RPM
        DogLog.log("Shooter/CurrentRPM", leftMotor.getVelocity().getValueAsDouble() * 60.0);
        DogLog.log("Shooter/TargetRPM", currentTargetRpm);
    }
}