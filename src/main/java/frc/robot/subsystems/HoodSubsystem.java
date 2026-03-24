package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import dev.doglog.DogLog;

public class HoodSubsystem extends SubsystemBase {
    private static final int HOOD_MOTOR_ID = 15; 
    private static final double GEAR_RATIO = 250.0;
    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final SparkClosedLoopController hoodPid;

    public HoodSubsystem() {
        hoodMotor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        double positionConversionFactor = 360.0 / GEAR_RATIO;
        config.encoder.positionConversionFactor(positionConversionFactor);
        config.closedLoop.pid(0.05, 0.0, 0.0);
        config.softLimit.forwardSoftLimit(70.0f).reverseSoftLimit(0.0f).forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
        hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hoodEncoder = hoodMotor.getEncoder();
        hoodPid = hoodMotor.getClosedLoopController();
    }

    public void setTargetAngle(Rotation2d angle) {
        hoodPid.setReference(angle.getDegrees(), ControlType.kPosition);
        DogLog.log("Hood/TargetDegrees", angle.getDegrees());
    }

    public boolean isAtAngle(Rotation2d targetAngle) {
        return Math.abs(hoodEncoder.getPosition() - targetAngle.getDegrees()) < 1.5; 
    }

    public double getCurrentAngleDegrees() {
        return hoodEncoder.getPosition();
    }

    @Override
    public void periodic() {
        DogLog.log("Hood/CurrentDegrees", hoodEncoder.getPosition());
    }
}