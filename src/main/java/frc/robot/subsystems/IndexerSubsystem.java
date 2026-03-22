package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    // TODO: Verify your CAN ID for the Indexer Falcon
    private static final int INDEXER_FALCON_ID = 30;

    private final TalonFX indexerMotor;

    // Pre-allocate a duty cycle (percent output) request to save loop times
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

    public IndexerSubsystem() {
        // The Falcon 500 also uses the TalonFX class in Phoenix 6
        indexerMotor = new TalonFX(INDEXER_FALCON_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // We typically want the indexer to brake so game pieces don't drift into the shooter prematurely
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        indexerMotor.getConfigurator().apply(config);
    }

    /**
     * Runs the indexer at a specified percentage output.
     * @param percentOutput Range from -1.0 (full reverse) to 1.0 (full forward)
     */
    public void setSpeed(double percentOutput) {
        indexerMotor.setControl(dutyCycleRequest.withOutput(percentOutput));
    }

    /**
     * Feeds the game piece into the shooter.
     */
    public void feedShooter() {
        // Adjust this speed based on your physical testing
        setSpeed(0.8);
    }

    /**
     * Feeds all balls into the shooter continuously.
     */
    public void feedAllBalls() {
        feedShooter();
    }

    /**
     * Reverses the indexer to clear jams or outtake game pieces.
     */
    public void reverse() {
        setSpeed(-0.5);
    }

    /**
     * Stops the indexer motor.
     */
    public void stop() {
        setSpeed(0.0);
    }

    /**
     * Stops the feeder motor (alias for stop() to match command usage).
     */
    public void stopFeeder() {
        stop();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You can add DogLog logging here if you want to track indexer speed/current
    }
}