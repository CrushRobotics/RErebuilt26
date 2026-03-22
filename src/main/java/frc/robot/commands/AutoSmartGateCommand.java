package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.BallisticSolver.FiringSolution;

public class AutoSmartGateCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final HoodSubsystem hood;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final Supplier<Pose3d> targetHubSupplier;
    private final BooleanSupplier isScoringPhase;

    public AutoSmartGateCommand(
            CommandSwerveDrivetrain drivetrain,
            HoodSubsystem hood,
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            Supplier<Pose3d> targetHubSupplier,
            BooleanSupplier isScoringPhase) {
        
        this.drivetrain = drivetrain;
        this.hood = hood;
        this.shooter = shooter;
        this.indexer = indexer;
        this.targetHubSupplier = targetHubSupplier;
        this.isScoringPhase = isScoringPhase;

        // CRITICAL: Notice we do NOT add the drivetrain as a requirement here!
        // This allows Pure Pursuit to drive the robot at the exact same time.
        addRequirements(hood, shooter, indexer);
    }

    @Override
    public void execute() {
        // 0. Check if we are retrieving fuel. If so, shut down the weapons and do nothing.
        if (!isScoringPhase.getAsBoolean()) {
            shooter.stop();
            indexer.stopFeeder();
            // Note: You could also add hood.setTargetAngle(STOW_ANGLE) here 
            // to keep the hood protected during intaking.
            return;
        }

        // 1. Fetch the 3D physics solution based on our current auto position
        FiringSolution solution = drivetrain.calculateFiringSolution(targetHubSupplier.get());

        if (solution == null) {
            indexer.stopFeeder();
            return;
        }

        // 2. Spool the weapons
        hood.setTargetAngle(solution.hoodAimAngle);
        shooter.setTargetVelocity(solution.shotVelocityMps);

        // 3. The Triple Gate Check
        boolean isChassisReady = drivetrain.isChassisAimed(solution.chassisAimAngle);
        boolean isHoodReady = hood.isAtAngle(solution.hoodAimAngle);
        boolean isShooterReady = shooter.isAtVelocity(solution.shotVelocityMps);

        // 4. Fire if perfectly aligned!
        if (isChassisReady && isHoodReady && isShooterReady) {
            indexer.feedAllBalls();
        } else {
            indexer.stopFeeder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        indexer.stopFeeder();
    }
}