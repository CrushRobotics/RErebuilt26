package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    // Hardware constraints
    private static final int SHOT_FEEDER_ID = 4; // 775 Pro
    private static final int GROUND_BARS_ID = 2; // NEO

    private final SparkMax shotFeederMotor;
    private final SparkMax groundBarsMotor;
    private final RelativeEncoder groundBarsEncoder;

    public IndexerSubsystem() {
        // Shot feeder is a VEX 775, which requires Brushed mode on a Spark Max
        shotFeederMotor = new SparkMax(SHOT_FEEDER_ID, MotorType.kBrushed);
        
        // Ground bars use a REV NEO, which requires Brushless mode
        groundBarsMotor = new SparkMax(GROUND_BARS_ID, MotorType.kBrushless);

        // --- Shot Feeder Configuration ---
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.idleMode(IdleMode.kBrake);
        feederConfig.inverted(false); 
        feederConfig.smartCurrentLimit(30); // Prevent battery voltage sag
        feederConfig.voltageCompensation(12.0); // Calibrate voltage
        
        // --- Ground Bars Configuration ---
        SparkMaxConfig groundBarsConfig = new SparkMaxConfig();
        groundBarsConfig.idleMode(IdleMode.kBrake);
        groundBarsConfig.smartCurrentLimit(30); // Prevent battery voltage sag
        groundBarsConfig.voltageCompensation(12.0); // Calibrate voltage
        
        // By inverting the ground bars in the configuration, we can send both motors 
        // the exact same speed command and they will physically spin in opposite directions.
        groundBarsConfig.inverted(true);

        // Apply configurations
        shotFeederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        groundBarsMotor.configure(groundBarsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // NEOs have a built-in encoder we can safely read for telemetry
        groundBarsEncoder = groundBarsMotor.getEncoder();
    }

    public void setSpeed(double percentOutput) {
        // Both receive the same value, but hardware inversion makes them spin oppositely
        shotFeederMotor.set(percentOutput);
        groundBarsMotor.set(percentOutput);
    }

    public void feedShooter() { setSpeed(0.8); }
    public void feedAllBalls() { feedShooter(); }
    
    // Reverses the feeder to unjam
    public void reverseFeeder() { setSpeed(-0.8); }
    
    public void stop() { setSpeed(0.0); }
    public void stopFeeder() { stop(); }

    public double getPositionRotations() {
        return groundBarsEncoder.getPosition();
    }

    @Override
    public void periodic() {}
}