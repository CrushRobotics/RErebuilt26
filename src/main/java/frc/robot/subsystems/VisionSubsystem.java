package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;

public class VisionSubsystem extends SubsystemBase {

    // -------------------------------------------------------------------------
    // TODO: Replace placeholder camera names with the actual hostnames from PhotonVision
    // -------------------------------------------------------------------------
    private static final String CAMERA_1_NAME = "Placeholder 1";
    private static final String CAMERA_2_NAME = "Placeholder 2";
    private static final String CAMERA_3_NAME = "Placeholder 3";

    // -------------------------------------------------------------------------
    // TODO: Measure and enter the exact 3D physical transforms of your cameras
    // Transform3d = robot center -> camera (x forward, y left, z up, in meters)
    // -------------------------------------------------------------------------
    private static final Transform3d CAMERA_1_TRANSFORM = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0)); 
    private static final Transform3d CAMERA_2_TRANSFORM = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0)); 
    private static final Transform3d CAMERA_3_TRANSFORM = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0)); 
    // -------------------------------------------------------------------------

    private final CommandSwerveDrivetrain drivetrain;

    private final PhotonCamera camera1 = new PhotonCamera(CAMERA_1_NAME);
    private final PhotonCamera camera2 = new PhotonCamera(CAMERA_2_NAME);
    private final PhotonCamera camera3 = new PhotonCamera(CAMERA_3_NAME);

    private final AprilTagFieldLayout fieldLayout;

    private final PhotonPoseEstimator estimator1;
    private final PhotonPoseEstimator estimator2;
    private final PhotonPoseEstimator estimator3;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Build AprilTagFieldLayout from FieldConstants map
        List<AprilTag> tags = new ArrayList<>();
        FieldConstants.APRIL_TAG_FIELD_LAYOUT.forEach((id, pose) ->
            tags.add(new AprilTag(id, pose))
        );
        fieldLayout = new AprilTagFieldLayout(
            tags,
            FieldConstants.FIELD_LENGTH_METERS,
            FieldConstants.FIELD_WIDTH_METERS
        );

        estimator1 = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CAMERA_1_TRANSFORM);
        estimator2 = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CAMERA_2_TRANSFORM);
        estimator3 = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CAMERA_3_TRANSFORM);

        estimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        estimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        estimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        updateEstimator(estimator1, camera1, "Camera1");
        updateEstimator(estimator2, camera2, "Camera2");
        updateEstimator(estimator3, camera3, "Camera3");
    }

    private void updateEstimator(PhotonPoseEstimator estimator, PhotonCamera camera, String label) {
        var results = camera.getAllUnreadResults();
        for (var result : results) {
            Optional<EstimatedRobotPose> estimated = estimator.update(result);
            estimated.ifPresent(est -> {
                // Reject wildly implausible poses
                var pose = est.estimatedPose.toPose2d();
                if (pose.getX() < 0 || pose.getX() > FieldConstants.FIELD_LENGTH_METERS) return;
                if (pose.getY() < 0 || pose.getY() > FieldConstants.FIELD_WIDTH_METERS) return;

                drivetrain.addVisionMeasurement(pose, est.timestampSeconds);

                DogLog.log("Vision/" + label + "/PoseX", pose.getX());
                DogLog.log("Vision/" + label + "/PoseY", pose.getY());
                DogLog.log("Vision/" + label + "/PoseYawDeg", pose.getRotation().getDegrees());
                DogLog.log("Vision/" + label + "/TagCount", est.targetsUsed.size());
            });
        }
    }
}