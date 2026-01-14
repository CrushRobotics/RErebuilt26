package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
public final class FieldConstants {

    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.22);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(316.64);

    // Official AprilTag field layout for REBUILT 2026
    // Coordinates derived from GE-26000 Series Field Drawings
    public static final Map<Integer, Pose3d> APRIL_TAG_FIELD_LAYOUT = Map.ofEntries(
        // --- RED ALLIANCE SIDE ---
        
        // ID 1: Trench
        Map.entry(1, new Pose3d(Units.inchesToMeters(467.64), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        // ID 2: Hub
        Map.entry(2, new Pose3d(Units.inchesToMeters(469.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(90)))),
        // ID 3: Hub2
        Map.entry(3, new Pose3d(Units.inchesToMeters(445.35), Units.inchesToMeters(172.84), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        // ID 4: Hub3
        Map.entry(4, new Pose3d(Units.inchesToMeters(445.35), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        // ID 5: Hub4
        Map.entry(5, new Pose3d(Units.inchesToMeters(469.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(270)))),
        // ID 6: Trench
        Map.entry(6, new Pose3d(Units.inchesToMeters(467.64), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        // ID 7: Trench
        Map.entry(7, new Pose3d(Units.inchesToMeters(470.59), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00), new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        // ID 8: Hub5
        Map.entry(8, new Pose3d(Units.inchesToMeters(483.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(270)))),
        // ID 9: Hub6
        Map.entry(9, new Pose3d(Units.inchesToMeters(492.88), Units.inchesToMeters(144.84), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        // ID 10: Hub7
        Map.entry(10, new Pose3d(Units.inchesToMeters(492.88), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        // ID 11: Hub8
        Map.entry(11, new Pose3d(Units.inchesToMeters(483.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(90)))),
        // ID 12: Trench
        Map.entry(12, new Pose3d(Units.inchesToMeters(470.59), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00), new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        // ID 13: Outpost1
        Map.entry(13, new Pose3d(Units.inchesToMeters(650.92), Units.inchesToMeters(291.47), Units.inchesToMeters(21.75), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        // ID 14: Outpost2
        Map.entry(14, new Pose3d(Units.inchesToMeters(650.92), Units.inchesToMeters(274.47), Units.inchesToMeters(21.75), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        // ID 15: Tower
        Map.entry(15, new Pose3d(Units.inchesToMeters(650.90), Units.inchesToMeters(170.22), Units.inchesToMeters(21.75), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        // ID 16: Tower
        Map.entry(16, new Pose3d(Units.inchesToMeters(650.90), Units.inchesToMeters(153.22), Units.inchesToMeters(21.75), new Rotation3d(0, 0, Units.degreesToRadians(180)))),

        // --- BLUE ALLIANCE SIDE -
        // ID 17
        Map.entry(17, new Pose3d(Units.inchesToMeters(183.59), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00), new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        // ID 18
        Map.entry(18, new Pose3d(Units.inchesToMeters(182.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(270)))),
        // ID 19
        Map.entry(19, new Pose3d(Units.inchesToMeters(205.87), Units.inchesToMeters(144.84), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        // ID 20
        Map.entry(20, new Pose3d(Units.inchesToMeters(205.87), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        // ID 21
        Map.entry(21, new Pose3d(Units.inchesToMeters(182.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(90)))),
        // ID 22
        Map.entry(22, new Pose3d(Units.inchesToMeters(183.59), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00), new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        // ID 23
        Map.entry(23, new Pose3d(Units.inchesToMeters(180.64), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        // ID 24
        Map.entry(24, new Pose3d(Units.inchesToMeters(168.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(90)))),
        // ID 25
        Map.entry(25, new Pose3d(Units.inchesToMeters(158.34), Units.inchesToMeters(172.84), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        // ID 26
        Map.entry(26, new Pose3d(Units.inchesToMeters(158.34), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        // ID 27:
        Map.entry(27, new Pose3d(Units.inchesToMeters(168.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25), new Rotation3d(0, 0, Units.degreesToRadians(270)))),
        // ID 28
        Map.entry(28, new Pose3d(Units.inchesToMeters(180.64), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00), new Rotation3d(0, 0, Units.degreesToRadians(180)))),
        // ID 29
        Map.entry(29, new Pose3d(Units.inchesToMeters(0.30), Units.inchesToMeters(26.22), Units.inchesToMeters(21.75), new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        // ID 30
        Map.entry(30, new Pose3d(Units.inchesToMeters(0.30), Units.inchesToMeters(43.22), Units.inchesToMeters(21.75), new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        // ID 31
        Map.entry(31, new Pose3d(Units.inchesToMeters(0.32), Units.inchesToMeters(147.47), Units.inchesToMeters(21.75), new Rotation3d(0, 0, Units.degreesToRadians(0)))),
        // ID 32
        Map.entry(32, new Pose3d(Units.inchesToMeters(0.32), Units.inchesToMeters(164.47), Units.inchesToMeters(21.75), new Rotation3d(0, 0, Units.degreesToRadians(0))))
    );
}