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
    
    // GEARING: Krakens to Shooter Wheels is 1:1.6 (Overdrive).
    // 1 rotation of the motor = 1.6 rotations of the wheel.
    private static final double WHEEL_ROTATIONS_PER_MOTOR_ROTATION = 1.6; 

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    // CTRE Phoenix 6 Velocity control request (Voltage-based is usually best for flywheels)
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    
    private double currentTargetRpm = 0.0;

    public ShooterSubsystem() {
        leftMotor = new TalonFX(LEFT_KRAKEN_ID);
        rightMotor = new TalonFX(RIGHT_KRAKEN_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // --- KRAKEN X60 HEAVY BRASS FLYWHEEL TUNING ---
        // MOI: 4 lbs*in^2 requires aggressive recovery but relies heavily on Feedforward
        
        // kS (Static Friction): Volts needed to just break static friction (usually 0.1 - 0.25)
        config.Slot0.kS = 0.15;
        
        // kV (Velocity Feedforward): Volts per RPS. 
        // A Kraken free spins at ~100 RPS at 12V. 12V / 100RPS = 0.12 V/RPS.
        config.Slot0.kV = 0.12; 
        
        // kP (Proportional): Added voltage per 1 RPS of error. 
        // Bumped from 0.11 to 0.5 to aggressively fight the high MOI during spin-up and recovery.
        config.Slot0.kP = 0.50; 
        
        config.Slot0.kI = 0.0; // Keep 0 for flywheels
        config.Slot0.kD = 0.0; // Rarely needed for flywheels (the heavy mass dampens oscillations naturally)

        // Make sure it coasts! Braking a brass flywheel will destroy your motor controllers.
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Optional but recommended for heavy flywheels: Add a stator current limit 
        // to prevent browning out the robot during the initial massive spin-up draw
        config.CurrentLimits.StatorCurrentLimit = 80.0; // Amps
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Aligned));
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
        // With heavy brass flywheels, you might be able to tighten this to 50 RPM once tuned.
        return Math.abs(currentRpm - targetRpm) < 100.0; 
    }

    public void stop() {
        leftMotor.setControl(new com.ctre.phoenix6.controls.DutyCycleOut(0));
        currentTargetRpm = 0;
    }
    
    /**
     * Gets the current rotations of the shooter wheel.
     * Used to spin the 3D mechanism in AdvantageScope.
     */
    public double getPositionRotations() {
        // Multiply motor rotations by the gear ratio to get true wheel rotations
        return leftMotor.getPosition().getValueAsDouble() * WHEEL_ROTATIONS_PER_MOTOR_ROTATION;
    }

    /**
     * Converts linear exit velocity (m/s) to target Motor RPM.
     */
    private double convertMpsToRpm(double metersPerSecond) {
        // Linear Velocity = RPM * Circumference / 60
        // Therefore: RPM = (Velocity * 60) / (PI * Diameter)
        double targetWheelRPM = (metersPerSecond * 60.0) / (Math.PI * WHEEL_DIAMETER_METERS);
        
        // Divide by the overdrive ratio to get the target MOTOR RPM
        // e.g., to spin the wheels at 4800 RPM, the motor only needs to spin at 3000 RPM
        return targetWheelRPM / WHEEL_ROTATIONS_PER_MOTOR_ROTATION;
    }

    @Override
    public void periodic() {
        // Log both the current actual RPM and the current Target RPM
        DogLog.log("Shooter/CurrentRPM", leftMotor.getVelocity().getValueAsDouble() * 60.0);
        DogLog.log("Shooter/TargetRPM", currentTargetRpm);
    }
}