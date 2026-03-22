package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.DogLog;

public class HoodSubsystem extends SubsystemBase {
    // TODO: Set your actual CAN ID for the Hood motor
    private static final int HOOD_MOTOR_ID = 15; 
    private static final double GEAR_RATIO = 250.0;

    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final SparkClosedLoopController hoodPid;

    public HoodSubsystem() {
        // Initialize NEO using the new 2025+ API
        hoodMotor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        
        // Create a new configuration object
        SparkMaxConfig config = new SparkMaxConfig();

        // CONVERSION FACTOR: 
        // 1 motor rotation = (1 / 250) of a mechanism rotation.
        // We multiply by 360 to work directly in Degrees.
        double positionConversionFactor = 360.0 / GEAR_RATIO;
        config.encoder.positionConversionFactor(positionConversionFactor);

        // TODO: Tune these PID values for your Hood
        config.closedLoop.pid(0.05, 0.0, 0.0);

        // TODO: VERY IMPORTANT - Set soft limits so you don't break your hood!
        // config.softLimit.forwardSoftLimit(60.0f); // Max degrees
        // config.softLimit.reverseSoftLimit(0.0f);  // Min degrees
        // config.softLimit.forwardSoftLimitEnabled(true);
        // config.softLimit.reverseSoftLimitEnabled(true);

        // Apply the configuration to the motor and burn to flash
        hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodEncoder = hoodMotor.getEncoder();
        hoodPid = hoodMotor.getClosedLoopController();
    }

    public void setTargetAngle(Rotation2d angle) {
        // Command the NEO to go to the specific degree
        hoodPid.setReference(angle.getDegrees(), ControlType.kPosition);
        DogLog.log("Hood/TargetDegrees", angle.getDegrees());
    }

    public boolean isAtAngle(Rotation2d targetAngle) {
        // The "Smart Gate" check: Are we within 1.5 degrees of the target?
        double currentAngle = hoodEncoder.getPosition();
        return Math.abs(currentAngle - targetAngle.getDegrees()) < 1.5; 
    }

    @Override
    public void periodic() {
        DogLog.log("Hood/CurrentDegrees", hoodEncoder.getPosition());
    }
}