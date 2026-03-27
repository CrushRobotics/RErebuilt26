package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.routines.FullBox;
import frc.robot.auto.routines.PureScore1;
import frc.robot.auto.routines.PureScore1PATH;
import frc.robot.auto.routines.PureScore2;
import frc.robot.auto.routines.TestAuto;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousLogic {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutonomousLogic(CommandSwerveDrivetrain drivetrain, HoodSubsystem hood, ShooterSubsystem shooter, IndexerSubsystem indexer) {

        // Register all routines
        autoChooser.setDefaultOption("PureScore1", new PureScore1(drivetrain, hood, shooter, indexer));
        autoChooser.addOption("PureScore1 PATH ONLY", new PureScore1PATH(drivetrain));
        autoChooser.addOption("PureScore2", new PureScore2(drivetrain, hood, shooter, indexer));
        autoChooser.addOption("FullBox", new FullBox(drivetrain, hood, shooter, indexer));
        autoChooser.addOption("Test Auto", new TestAuto(drivetrain, hood, shooter, indexer));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}