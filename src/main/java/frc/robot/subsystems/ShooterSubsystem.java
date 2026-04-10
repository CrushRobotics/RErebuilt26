package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX topMotor    = new TalonFX(12, "CrushSwerve");
    private final TalonFX bottomMotor = new TalonFX(13, "CrushSwerve");

    // Raw voltage control request
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    // kS = voltage to overcome static friction
    // kV = voltage to hold 1 m/s
    private final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(0.1, 0.2);

    // PID trims error that feedforward misses
    private final PIDController pid =
        new PIDController(0.1, 0.0, 0.0);

    private double targetVelocityMps = 0.0;

    // --- NEW: Voltage ramp rate limiter ---
    // Units: Volts per second
    // This directly controls spin-up aggressiveness.
    //
    // Recommended tuning:
    // 8.0  = fast (~1.5s spin-up)
    // 6.0  = safe competition default
    // 4.0  = very gentle (lowest current spike)
    //
    private static final double VOLTAGE_RAMP_RATE = 6.0;

    private final SlewRateLimiter voltageRampLimiter =
        new SlewRateLimiter(VOLTAGE_RAMP_RATE);

    // Update this to match your actual wheel diameter
    // Default: 4 inch wheel
    private static final double WHEEL_CIRCUMFERENCE_METERS =
        0.1016 * Math.PI;

    public ShooterSubsystem() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode =
            NeutralModeValue.Coast;

        config.MotorOutput.Inverted =
            InvertedValue.Clockwise_Positive;

        // --- Current Limits ---
        // Stator = torque limit
        // Supply = battery protection

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        topMotor.getConfigurator().apply(config);
        bottomMotor.getConfigurator().apply(config);
    }

    /** Set the target surface velocity for both flywheels in meters per second. */
    public void setTargetVelocity(double velocityMps) {
        targetVelocityMps = velocityMps;
    }

    /** Immediately cuts power and coasts both motors to a stop. */
    public void stop() {

        targetVelocityMps = 0.0;

        // --- NEW: reset ramp limiter ---
        voltageRampLimiter.reset(0.0);

        topMotor.setControl(
            voltageRequest.withOutput(0.0)
        );

        bottomMotor.setControl(
            voltageRequest.withOutput(0.0)
        );
    }

    /** Returns the top motor position in rotations. */
    public double getPositionRotations() {
        return topMotor
            .getPosition()
            .getValueAsDouble();
    }

    /** Returns the top flywheel surface velocity in meters per second. */
    public double getTopVelocityMps() {
        return topMotor
            .getVelocity()
            .getValueAsDouble()
            * WHEEL_CIRCUMFERENCE_METERS;
    }

    /** Returns the bottom flywheel surface velocity in meters per second. */
    public double getBottomVelocityMps() {
        return bottomMotor
            .getVelocity()
            .getValueAsDouble()
            * WHEEL_CIRCUMFERENCE_METERS;
    }

    /**
     * Returns true when both flywheels are within tolerance of the requested speed.
     */
    public boolean isAtVelocity(double velocityMps) {

        final double toleranceMps = 1.0;

        boolean topOk =
            Math.abs(
                getTopVelocityMps()
                - velocityMps
            ) <= toleranceMps;

        boolean bottomOk =
            Math.abs(
                getBottomVelocityMps()
                - velocityMps
            ) <= toleranceMps;

        return topOk && bottomOk;
    }

    @Override
    public void periodic() {

        // Coast to stop if no velocity requested
        if (targetVelocityMps == 0.0) {

            topMotor.setControl(
                voltageRequest.withOutput(0.0)
            );

            bottomMotor.setControl(
                voltageRequest.withOutput(0.0)
            );

            SmartDashboard.putNumber(
                "Shooter/CommandedVolts",
                0.0
            );

            return;
        }

        // 1. Feedforward
        double ffVolts =
            feedforward.calculate(
                targetVelocityMps
            );

        // 2. PID correction
        double pidVolts =
            pid.calculate(
                getTopVelocityMps(),
                targetVelocityMps
            );

        // 3. Clamp to battery limits
        double rawVolts =
            MathUtil.clamp(
                ffVolts + pidVolts,
                -12.0,
                12.0
            );

        // --- NEW: Apply ramp limiting ---
        double totalVolts =
            voltageRampLimiter.calculate(
                rawVolts
            );

        // 4. Apply to motors
        topMotor.setControl(
            voltageRequest.withOutput(
                totalVolts
            )
        );

        bottomMotor.setControl(
            voltageRequest.withOutput(
                totalVolts
            )
        );

        // --- SmartDashboard telemetry ---

        SmartDashboard.putNumber(
            "Shooter/CommandedVolts",
            totalVolts
        );

        SmartDashboard.putNumber(
            "Shooter/TargetVelocityMps",
            targetVelocityMps
        );

        SmartDashboard.putNumber(
            "Shooter/TopVelocityMps",
            getTopVelocityMps()
        );

        SmartDashboard.putNumber(
            "Shooter/BottomVelocityMps",
            getBottomVelocityMps()
        );

        SmartDashboard.putNumber(
            "Shooter/TopCurrentAmps",
            topMotor
                .getStatorCurrent()
                .getValueAsDouble()
        );

        SmartDashboard.putNumber(
            "Shooter/BottomCurrentAmps",
            bottomMotor
                .getStatorCurrent()
                .getValueAsDouble()
        );

        SmartDashboard.putBoolean(
            "Shooter/AtTargetSpeed",
            isAtVelocity(targetVelocityMps)
        );

        // --- DogLog telemetry ---

        DogLog.log(
            "Shooter/TargetVelocityMps",
            targetVelocityMps
        );

        DogLog.log(
            "Shooter/TopVelocityMps",
            getTopVelocityMps()
        );

        DogLog.log(
            "Shooter/BottomVelocityMps",
            getBottomVelocityMps()
        );

        DogLog.log(
            "Shooter/AppliedVolts",
            totalVolts
        );

        DogLog.log(
            "Shooter/IsAtTargetSpeed",
            isAtVelocity(targetVelocityMps)
        );
    }
}