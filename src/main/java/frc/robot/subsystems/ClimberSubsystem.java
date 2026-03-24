package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private double simulatedExtensionMeters = 0.0;

    public ClimberSubsystem() {}

    public double getExtensionMeters() {
        return simulatedExtensionMeters;
    }

    @Override
    public void periodic() {}
}