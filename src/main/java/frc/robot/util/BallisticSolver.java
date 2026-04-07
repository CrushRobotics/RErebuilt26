package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class BallisticSolver {
    
    // Maps distance to the target (in meters) -> to Target Shooter Velocity (M/s or RPM)
    private static final InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();
    
    // Maps distance to the target (in meters) -> to Target Hood Angle (Degrees)
    private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    static {
        // ==========================================
        // TODO: TUNE THESE VALUES ON THE REAL ROBOT!
        // Measure your distance from the target goal, find the perfect shot, and add it here.
        // The robot will automatically interpolate smoothly between these distances.
        // ==========================================
        
        // Format: .put(Distance_In_Meters, Target_Value);
        
        // --- Velocity Map (Distance -> Speed) ---
        velocityMap.put(1.3, 10.0); // Point-blank / Bumper-to-goal shot
        velocityMap.put(2.0, 12.5); // Short range
        velocityMap.put(3.0, 15.0); // Mid range
        velocityMap.put(4.0, 18.0); // Far shot
        velocityMap.put(5.5, 22.0); // Very far / Cross-zone shot

        // --- Hood Map (Distance -> Angle Degrees) ---
        hoodMap.put(1.3, 15.0); // Lower angle for point-blank shots
        hoodMap.put(2.0, 25.0);
        hoodMap.put(3.0, 35.0);
        hoodMap.put(4.0, 42.0);
        hoodMap.put(5.5, 48.0); // Higher angle to arc it in from far away
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
     * Calculates the perfect aiming configuration based on your distance to the target
     * and compensates for your current driving velocity.
     */
    public static FiringSolution solveShot(Pose2d robotPose, Pose3d targetPose, double vx, double vy, double shooterHeight) {
        // 1. Calculate direct distance to the target (2D distance)
        Translation2d targetTranslation = new Translation2d(targetPose.getX(), targetPose.getY());
        double distanceToTarget = robotPose.getTranslation().getDistance(targetTranslation);

        // 2. Velocity Compensation (Shoot-on-the-move calculation)
        // Approximate the time the game piece is in the air (Distance / Avg Projectile Speed)
        double approximateTimeOfFlight = distanceToTarget / 15.0; 
        
        // Calculate the "Virtual Target" - offsetting our aim by how much we will move while the projectile flies
        Translation2d movingOffset = new Translation2d(vx * approximateTimeOfFlight, vy * approximateTimeOfFlight);
        Translation2d virtualTarget = targetTranslation.minus(movingOffset);

        // 3. Calculate Chassis Aim Angle to face the adjusted Virtual Target
        Rotation2d chassisAimAngle = new Rotation2d(
            virtualTarget.getX() - robotPose.getX(),
            virtualTarget.getY() - robotPose.getY()
        );

        // 4. Look up dynamic Shooter Velocity and Hood Angle based on our actual distance
        double targetVelocity = velocityMap.get(distanceToTarget);
        double targetHoodAngle = hoodMap.get(distanceToTarget);

        return new FiringSolution(chassisAimAngle, targetHoodAngle, targetVelocity);
    }
}