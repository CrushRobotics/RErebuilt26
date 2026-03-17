package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.auto.routines.FullBox;
import frc.robot.auto.routines.PureScore1;
import frc.robot.auto.routines.PureScore2;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonomousLogic {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final CommandSwerveDrivetrain drivetrain;

    public AutonomousLogic(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Register our separated autonomous routines
        autoChooser.setDefaultOption("PureScore1", new PureScore1(drivetrain));
        
        autoChooser.addOption("PureScore2", new PureScore2(drivetrain));
        autoChooser.addOption("FullBox", new FullBox(drivetrain));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}