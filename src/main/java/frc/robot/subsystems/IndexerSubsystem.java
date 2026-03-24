package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private static final int INDEXER_FALCON_ID = 30;
    private final TalonFX indexerMotor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

    public IndexerSubsystem() {
        indexerMotor = new TalonFX(INDEXER_FALCON_ID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerMotor.getConfigurator().apply(config);
    }

    public void setSpeed(double percentOutput) {
        indexerMotor.setControl(dutyCycleRequest.withOutput(percentOutput));
    }

    public void feedShooter() { setSpeed(0.8); }
    public void feedAllBalls() { feedShooter(); }
    public void stop() { setSpeed(0.0); }
    public void stopFeeder() { stop(); }

    public double getPositionRotations() {
        return indexerMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {}
}