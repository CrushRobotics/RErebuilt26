package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private static final int INTAKE_PIVOT_ID = 3;
    private static final int INTAKE_MOTOR_ID = 41; 
    
    // --- Setpoints (TUNE THESE ON THE REAL ROBOT!) ---
    private static final double DEPLOYED_POSITION_ROTATIONS = -16.404; // The encoder value when fully down
    private static final double RETRACTED_POSITION_ROTATIONS = 0.0;  // The encoder value when stowed
    private static final double HALF_POSITION_ROTATIONS = -9.833; // A halfway point for testing and tuning
    
    // --- Testing Configuration ---
    // Use this extremely low limit (e.g., 5% output) when testing the 111:1 gear ratio
    private static final double MAX_TEST_OUTPUT = 1.5; 
    
    private final SparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotPid;
    
    // The Falcon 500 motor that actually spins to intake the game pieces
    private final TalonFX intakeMotor;

    public IntakeSubsystem() {
        // --- Pivot Motor Initialization (REV NEO/SparkMax) ---
        pivotMotor = new SparkMax(INTAKE_PIVOT_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Brake mode helps hold the heavy intake bar in place when you stop powering it
        config.idleMode(IdleMode.kBrake);
        
        // --- PID Constants ---
        config.closedLoop.pid(1.5, 0.05, 0.0);
        
        // --- Output Limits ---
        // We set the absolute maximum voltage the PID is allowed to send to the motor.
        // This is a critical safety feature when tuning high-reduction gearboxes.
        config.closedLoop.outputRange(-MAX_TEST_OUTPUT, MAX_TEST_OUTPUT);
        
        // --- Trapezoidal PID Configuration (REV MAXMotion) ---
        // This generates a smooth trapezoidal curve during movement (Profiled PID)
        config.closedLoop.maxMotion.maxVelocity(1800); // Drastically lowered for testing
        config.closedLoop.maxMotion.maxAcceleration(2000); // Drastically lowered for testing
        config.closedLoop.maxMotion.allowedClosedLoopError(0.01);
        
        // --- Optional Soft Limits ---
        // If you want to prevent the intake from smashing into the floor or the robot frame,
        // figure out the encoder positions for "Up" and "Down", then uncomment and tune these!
        // config.softLimit.forwardSoftLimit(15.0f).forwardSoftLimitEnabled(true);
        // config.softLimit.reverseSoftLimit(0.0f).reverseSoftLimitEnabled(true);

        // Apply configuration to the hardware
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        pivotEncoder = pivotMotor.getEncoder();
        pivotPid = pivotMotor.getClosedLoopController();
        
        // --- Intake Roller Motor Initialization (CTRE TalonFX/Falcon 500) ---
        // Configured to run on the "CrushSwerve" CANbus
        intakeMotor = new TalonFX(INTAKE_MOTOR_ID, "CrushSwerve");
        // Set to Coast mode so game pieces don't get stuck if the robot dies
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Drives the intake to a specific position using the Trapezoidal profile.
     * @param targetRotations Position to move to.
     */
    public void setTargetPosition(double targetRotations) {
        pivotPid.setReference(targetRotations, ControlType.kMAXMotionPositionControl);
        DogLog.log("Intake/TargetRotations", targetRotations);
    }

    /**
     * Directly sets the motor power for the intake pivot (Open loop fallback).
     * @param speed Range from -1.0 to 1.0
     */
    public void setPivotSpeed(double speed) {
        // Cap the open loop speed as well for safety
        double clampedSpeed = Math.max(-MAX_TEST_OUTPUT, Math.min(MAX_TEST_OUTPUT, speed));
        pivotMotor.set(clampedSpeed);
    }
    
    /**
     * Spins the intake rollers to pull a game piece in.
     */
    public void runIntakeRollers() {
        intakeMotor.set(1.0); // Tune this speed as needed
    }
    
    /**
     * Spins the intake rollers backwards to spit a game piece out.
     */
    public void reverseIntakeRollers() {
        intakeMotor.set(-1.0);
    }
    
    /**
     * Stops the intake rollers from spinning.
     */
    public void stopIntakeRollers() {
        intakeMotor.set(0.0);
    }
    
    /**
     * Moves the intake down towards the floor using the profiled PID.
     */
    public void deploy() {
        setTargetPosition(DEPLOYED_POSITION_ROTATIONS); 
    }
    
    /**
     * Moves the intake back up into the robot using the profiled PID.
     */
    public void retract() {
        setTargetPosition(RETRACTED_POSITION_ROTATIONS); 
    }

    /**
     * Moves the intake to the halfway point using the profiled PID.
     */
    public void halfDeploy() {
        setTargetPosition(HALF_POSITION_ROTATIONS);
    }

    /**
     * Instantly stops both the pivot motor and the intake rollers.
     */
    public void stopAll() {
        pivotMotor.stopMotor();
        stopIntakeRollers();
    }

    /**
     * Returns the current encoder rotations of the pivot.
     */
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // Stream the position and power to AdvantageScope/DogLog so you can tune your limits
        DogLog.log("Intake/PivotRotations", getPivotPosition());
        DogLog.log("Intake/PivotAppliedOutput", pivotMotor.getAppliedOutput());
        DogLog.log("Intake/RollerAppliedOutput", intakeMotor.get());
        
        // Output to SmartDashboard for real-time viewing and tuning
        SmartDashboard.putNumber("Intake/PivotRotations", getPivotPosition());
    }
}