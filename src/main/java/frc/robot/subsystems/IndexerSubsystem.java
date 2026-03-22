package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    
    // TODO: Define your indexer/feeder motor here

    public IndexerSubsystem() {
        // Motor config goes here
    }

    /**
     * Turns on the indexer to push fuel into the shooter.
     */
    public void feedAllBalls() {
        // TODO: Set motor to 100% or desired feeding speed
    }

    /**
     * Halts the fuel immediately.
     */
    public void stopFeeder() {
        // TODO: Set motor to 0%
    }
}