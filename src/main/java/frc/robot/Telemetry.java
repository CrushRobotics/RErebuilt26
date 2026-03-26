package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    private final double MaxSpeed;

    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    
    // NATIVE WPILIB 3D PUBLISHERS
    // ChassisPose3D: Single pose for the main robot base
    private final StructPublisher<Pose3d> chassisPose3dPub = driveStateTable.getStructTopic("ChassisPose3D", Pose3d.struct).publish();
    // ComponentPoses3D: Array of poses for the moving mechanisms
    private final StructArrayPublisher<Pose3d> componentPoses3dPub = driveStateTable.getStructArrayTopic("ComponentPoses3D", Pose3d.struct).publish();
    
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1), new Mechanism2d(1, 1),
    };
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5).append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5).append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5).append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5).append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];
    
    // Store the last 2D pose so the 3D mechanism visualizer knows where the robot is globally
    private Pose2d m_lastPose = new Pose2d();

    public void telemeterize(SwerveDriveState state) {
        m_lastPose = state.Pose;
        
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }

    /** * Telemeterize 3D mechanisms to AdvantageScope 
     * Model 0: Hood
     * Model 1: Indexer
     * Model 2: Shooter
     * Model 3: Climber
     */
    public void telemeterizeMechanisms(double hoodDegrees, double indexerRots, double shooterRots, double climberExtMeters) {
        // Base chassis pose globally on the field
        Pose3d chassisPose = new Pose3d(m_lastPose);

        // Transform relative component positions into absolute field positions 
        // This ensures they never draw at the center of the field by accident!
        
        // 0. Hood pivots on the Y-axis (Pitch)
        Pose3d hoodPose = chassisPose.transformBy(new Transform3d(
            new Translation3d(0.0, 0.0, 0.5), 
            new Rotation3d(0.0, Math.toRadians(hoodDegrees), 0.0)
        ));
        
        // 1. Indexer roller spins continuously 
        Pose3d indexerPose = chassisPose.transformBy(new Transform3d(
            new Translation3d(0.0, 0.0, 0.2), 
            new Rotation3d(0.0, indexerRots * 2 * Math.PI, 0.0)
        ));

        // 2. Shooter flywheels spin continuously
        Pose3d shooterPose = chassisPose.transformBy(new Transform3d(
            new Translation3d(0.0, 0.0, 0.4), 
            new Rotation3d(0.0, shooterRots * 2 * Math.PI, 0.0)
        ));

        // 3. Climber translates linearly upwards (Z-axis offset + actual extension)
        Pose3d climberPose = chassisPose.transformBy(new Transform3d(
            new Translation3d(-0.2, 0.0, 0.3 + climberExtMeters), 
            new Rotation3d()
        ));

        Pose3d[] componentPoses = new Pose3d[] {hoodPose, indexerPose, shooterPose, climberPose};

        // Publish cleanly separated objects to NetworkTables for live 3D viewing
        chassisPose3dPub.set(chassisPose);
        componentPoses3dPub.set(componentPoses);
        
        // Log to CTRE SignalLogger for post-match
        SignalLogger.writeStruct("DriveState/ChassisPose3D", Pose3d.struct, chassisPose);
        SignalLogger.writeStructArray("DriveState/ComponentPoses3D", Pose3d.struct, componentPoses);
    }
}