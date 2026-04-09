package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class BallisticSolver {
    
    private static final InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    static {
        // --- Velocity Map (Distance in Meters -> Speed in Mps) ---
        // Added 0.0 bound to ensure values are defined if the robot is touching the hub
        velocityMap.put(0.0, 10.0); 
        velocityMap.put(1.3, 10.0);
        velocityMap.put(2.0, 12.5);
        velocityMap.put(3.0, 15.0);
        velocityMap.put(4.0, 18.0);
        velocityMap.put(5.5, 22.0);

        // --- Hood Map (Distance in Meters -> Angle in Degrees) ---
        // Added 0.0 bound to ensure values are defined
        hoodMap.put(0.0, 15.0);
        hoodMap.put(1.3, 15.0);
        hoodMap.put(2.0, 25.0);
        hoodMap.put(3.0, 35.0);
        hoodMap.put(4.0, 42.0);
        hoodMap.put(5.5, 48.0);
    }

    public static class FiringSolution {
        public Rotation2d chassisAimAngle;
        public double hoodAimAngle;
        public double shotVelocityMps;

        public FiringSolution(Rotation2d chassisAimAngle, double hoodAimAngle, double shotVelocityMps) {
            this.chassisAimAngle = chassisAimAngle;
            this.hoodAimAngle = hoodAimAngle;
            this.shotVelocityMps = shotVelocityMps;
        }
    }

    /**
     * Calculates the perfect aiming configuration based on field location.
     */
    public static FiringSolution solveShot(Pose2d robotPose, Pose3d targetPose, double vx, double vy, double shooterHeight) {
        // Calculate direct 2D distance to the target hub
        Translation2d targetTranslation = new Translation2d(targetPose.getX(), targetPose.getY());
        double distanceToTarget = robotPose.getTranslation().getDistance(targetTranslation);

        // Velocity Compensation (approximate 0.5s flight time for offset)
        double approximateTimeOfFlight = distanceToTarget / 15.0; 
        Translation2d movingOffset = new Translation2d(vx * approximateTimeOfFlight, vy * approximateTimeOfFlight);
        Translation2d virtualTarget = targetTranslation.minus(movingOffset);

        // Calculate Chassis Aim Angle
        Rotation2d chassisAimAngle = new Rotation2d(
            virtualTarget.getX() - robotPose.getX(),
            virtualTarget.getY() - robotPose.getY()
        );

        // Interpolate Shooter Velocity and Hood Angle from distance maps
        double targetVelocity = velocityMap.get(distanceToTarget);
        double targetHoodAngle = hoodMap.get(distanceToTarget);

        return new FiringSolution(chassisAimAngle, targetHoodAngle, targetVelocity);
    }
}