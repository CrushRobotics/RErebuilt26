package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    public ShooterSubsystem() {
    }

    public void setTargetVelocity(double velocityMps) {
        // TODO: Convert Meters Per Second exit velocity to Flywheel RPM 
        // using your wheel diameter and gearing!
    }

    public boolean isAtVelocity(double targetVelocityMps) {
        // TODO: Check if flywheels are spun up
        return true; 
    }

    public void stop() {
    }
}