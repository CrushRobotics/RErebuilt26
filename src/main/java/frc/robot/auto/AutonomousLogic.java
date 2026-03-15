package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Handles the configuration and selection of autonomous routines.
 * This class owns the SendableChooser and defines the specific paths used during Auto.
 */
public class AutonomousLogic {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final CommandSwerveDrivetrain drivetrain;

    public AutonomousLogic(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // --- 1. Test Pure Pursuit (Hardcoded Waypoints) ---
        // This makes a simple "L" shape on the field.
        autoChooser.setDefaultOption("Test Pure Pursuit (L-Shape)", 
            this.drivetrain.getPurePursuitCommand(
                List.of(
                    new Translation2d(0, 0),    // Start point
                    new Translation2d(2.0, 0),  // Drive forward 2 meters
                    new Translation2d(2.0, 2.0) // Strafe left 2 meters
                ),
                Rotation2d.fromDegrees(0) // Keep facing forward (0 degrees)
            )
        );

        // --- 2. Test Choreo Path ---
        // IMPORTANT: You must create a path named "TestPath" in the Choreo App and hit Export first.
        // It should save to src/main/deploy/choreo/TestPath.traj
        autoChooser.addOption("Test Choreo (TestPath.traj)", 
            this.drivetrain.getChoreoCommand("TestPath")
        );

        // Publish the chooser to the dashboard for the driver to see
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Returns the command currently selected in the dashboard dropdown.
     */
    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}