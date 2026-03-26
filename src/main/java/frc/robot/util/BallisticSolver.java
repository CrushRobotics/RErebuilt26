package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BallisticSolver {

    // --- ROBOT DIMENSIONS / OFFSETS ---
    // User: "Shooter is in the center of the robot, aligned to the back. Intake/Shooter face the same way."
    // This means the exit point is physically behind the center, but fires FORWARD.
    private static final double SHOOTER_OFFSET_X = -0.35; 
    private static final double SHOOTER_OFFSET_Y = 0.0; 

    public static class FiringSolution {
        public final double shotVelocityMps;
        public final Rotation2d hoodAimAngle;
        public final Rotation2d chassisAimAngle;

        public FiringSolution(double velocity, Rotation2d hoodAngle, Rotation2d chassisAngle) {
            this.shotVelocityMps = velocity;
            this.hoodAimAngle = hoodAngle;
            this.chassisAimAngle = chassisAngle;
        }
    }

    /**
     * Solves for the required robot heading and shooter parameters.
     * @param robotVx Field-relative X velocity (m/s)
     * @param robotVy Field-relative Y velocity (m/s)
     */
    public static FiringSolution solveShot(Pose2d robotPose, Pose3d targetHub, double robotVx, double robotVy, double shooterHeight) {
        // 1. Initial distance from center
        double centerDx = targetHub.getX() - robotPose.getX();
        double centerDy = targetHub.getY() - robotPose.getY();
        double groundDistanceCenter = Math.sqrt(centerDx * centerDx + centerDy * centerDy);
        
        // 2. Realistic Exit Velocity (Typical FRC is 12-18 mps)
        double exitVelocity = 15.0; 
        
        // Flight time estimate
        double flightTime = groundDistanceCenter / (exitVelocity * 0.9);

        // 3. VIRTUAL TARGET (Ballistic Compensation)
        // We subtract the robot's motion from the goal's position. 
        // If we drive right, we aim left of the hub.
        double virtualX = targetHub.getX() - (robotVx * flightTime);
        double virtualY = targetHub.getY() - (robotVy * flightTime);
        Translation2d virtualTarget = new Translation2d(virtualX, virtualY);

        // 4. SHOOTER OFFSET COMPENSATION
        // Since the shooter/intake face the SAME direction, chassis heading = shooter heading.
        Rotation2d angleToTarget = new Rotation2d(virtualX - robotPose.getX(), virtualY - robotPose.getY());
        
        // Position of the shooter exit in the field frame
        Translation2d shooterOffsetField = new Translation2d(SHOOTER_OFFSET_X, SHOOTER_OFFSET_Y).rotateBy(angleToTarget);
        Translation2d shooterPosField = robotPose.getTranslation().plus(shooterOffsetField);

        // Final vector from shooter exit to virtual target
        double finalDx = virtualX - shooterPosField.getX();
        double finalDy = virtualY - shooterPosField.getY();
        Rotation2d finalChassisHeading = new Rotation2d(finalDx, finalDy);

        // 5. Hood Angle based on distance from ACTUAL shooter exit
        double finalDistance = shooterPosField.getDistance(virtualTarget);
        Rotation2d hoodAngle = Rotation2d.fromDegrees(32.0 + (finalDistance * 1.95)); 

        return new FiringSolution(exitVelocity, hoodAngle, finalChassisHeading);
    }
}