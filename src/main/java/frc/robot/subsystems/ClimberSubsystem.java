package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private static final int CLIMBER_MOTOR_ID = 15;
    
    private final SparkMax climberMotor;
    private final RelativeEncoder encoder;
    
    // Example ratio: 25:1 gearbox, spool diameter 1.5 inches
    // Adjust these to match your physical robot!
    private static final double GEAR_RATIO = 25.0;
    private static final double SPOOL_CIRCUMFERENCE_METERS = 0.0381 * Math.PI; // 1.5 inch dia
    
    public ClimberSubsystem() {
        climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Brake mode is critical here so the robot doesn't instantly fall when you release the trigger
        config.idleMode(IdleMode.kBrake); 
        config.smartCurrentLimit(40); // Climbers need torque, but protect the motor from burning out
        config.voltageCompensation(12.0); // Added voltage compensation
        
        // Convert rotations to linear meters for telemetry
        config.encoder.positionConversionFactor(SPOOL_CIRCUMFERENCE_METERS / GEAR_RATIO);
        config.encoder.velocityConversionFactor((SPOOL_CIRCUMFERENCE_METERS / GEAR_RATIO) / 60.0);
        
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        encoder = climberMotor.getEncoder();
        
        encoder.setPosition(0);
    }

    /**
     * Directly sets the motor power output for manual control.
     * @param power Power from -1.0 to 1.0
     */
    public void setPower(double power) {
        climberMotor.set(power);
    }

    public double getExtensionMeters() {
        return encoder.getPosition();
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    @Override
    public void periodic() {
        DogLog.log("Climber/CurrentMeters", getExtensionMeters());
        DogLog.log("Climber/OutputAmps", climberMotor.getOutputCurrent());
        DogLog.log("Climber/AppliedOutput", climberMotor.getAppliedOutput());
    }
}