package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    
    // TODO: Define your Hood Motor/Servo here

    public HoodSubsystem() {
    }

    public void setTargetAngle(Rotation2d angle) {
        // TODO: Map the mathematical angle to your servo/motor position
    }

    public boolean isAtAngle(Rotation2d targetAngle) {
        // TODO: Replace with absolute encoder check
        // Example: return Math.abs(currentAngle.getDegrees() - targetAngle.getDegrees()) < 1.5;
        return true; 
    }
}