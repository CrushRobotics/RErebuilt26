package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonomousLogic {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final CommandSwerveDrivetrain drivetrain;

    public AutonomousLogic(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // --- 1. Test Pure Pursuit ---
        autoChooser.setDefaultOption("Test Pure Pursuit (L-Shape)", 
            this.drivetrain.getPurePursuitCommand(
                List.of(
                    new Translation2d(0, 0),
                    new Translation2d(2.0, 0),
                    new Translation2d(2.0, 2.0)
                ),
                Rotation2d.fromDegrees(0)
            )
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}