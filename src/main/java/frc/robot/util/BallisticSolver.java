package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BallisticSolver {
    private static final double GRAVITY = 9.81;

    public static class FiringSolution {
        public final Rotation2d chassisAimAngle;
        public final Rotation2d hoodAimAngle;
        public final double shotVelocityMps;

        public FiringSolution(Rotation2d chassisAimAngle, Rotation2d hoodAimAngle, double shotVelocityMps) {
            this.chassisAimAngle = chassisAimAngle;
            this.hoodAimAngle = hoodAimAngle;
            this.shotVelocityMps = shotVelocityMps;
        }
    }

    // Solve F(t) = 0 using Newton's method
    private static Double solveTimeNewton(double dx, double dy, double dz, double vrx, double vry, double v0, double t0) {
        double t = t0;
        for (int i = 0; i < 40; i++) {
            double Ft = F(t, dx, dy, dz, vrx, vry, v0);
            if (Math.abs(Ft) < 1e-6) return t;

            double dFt = dFdt(t, dx, dy, dz, vrx, vry, v0);
            if (Math.abs(dFt) < 1e-9) return null;

            t = t - Ft / dFt;
            if (t <= 0) return null;
        }
        return null;
    }

    private static double F(double t, double dx, double dy, double dz, double vrx, double vry, double v0) {
        if (t <= 0) return 1e9;
        double vzTerm = (dz + 0.5 * GRAVITY * t * t) / (v0 * t);
        double hx = dx - vrx * t;
        double hy = dy - vry * t;
        double R2 = hx * hx + hy * hy;
        return v0 * v0 * (1 - vzTerm * vzTerm) - R2 / (t * t);
    }

    private static double dFdt(double t, double dx, double dy, double dz, double vrx, double vry, double v0) {
        double h = 1e-5;
        return (F(t + h, dx, dy, dz, vrx, vry, v0) - F(t - h, dx, dy, dz, vrx, vry, v0)) / (2 * h);
    }

    public static FiringSolution solveShot(Pose2d robotPose, Pose3d goalPose, double robotVx, double robotVy, double shooterHeightMeters) {
        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();
        double dz = goalPose.getZ() - shooterHeightMeters;

        // Dynamic launch speed (can interpolate this for finer control)
        double dist = Math.hypot(dx, dy);
        double launchSpeed = (dist > 3.0) ? 15.0 : 10.0;

        Double t = solveTimeNewton(dx, dy, dz, robotVx, robotVy, launchSpeed, 1.0);
        if (t == null) return null; // No physical solution found

        // Azimuth (Chassis angle to target, compensating for velocity)
        double phi = Math.atan2(dy - robotVy * t, dx - robotVx * t);

        // Elevation (Hood Angle)
        double sinTheta = (dz + 0.5 * GRAVITY * t * t) / (launchSpeed * t);
        if (sinTheta > 1 || sinTheta < -1) return null;
        double theta = Math.asin(sinTheta);

        return new FiringSolution(Rotation2d.fromRadians(phi), Rotation2d.fromRadians(theta), launchSpeed);
    }
}