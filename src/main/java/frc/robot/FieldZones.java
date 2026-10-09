package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Where the trenches are (2026 REBUILT, blue-origin meters, from team 6328's public field constants). */
public final class FieldZones {
    private FieldZones() {}

    private static final double kFieldLength = 16.54;
    private static final double kFieldWidth = 8.07;
    /** Trench footprint, measured from our own alliance wall (same on both alliances) */
    private static final double kTrenchNearX = 4.03;
    private static final double kTrenchFarX = 5.22;
    /** The trench (opening + its inner support) runs this far in from each side wall */
    private static final double kTrenchFromSideWall = 1.67;

    /** @return true if the point is inside any of the 4 trenches, grown by margin meters on every side */
    public static boolean isNearTrench(Translation2d point, double margin) {
        double x = point.getX();
        double y = point.getY();
        boolean besideSideWall = y >= kFieldWidth - kTrenchFromSideWall - margin || y <= kTrenchFromSideWall + margin;
        boolean inBlueTrenchX = x >= kTrenchNearX - margin && x <= kTrenchFarX + margin;
        boolean inRedTrenchX = x >= kFieldLength - kTrenchFarX - margin && x <= kFieldLength - kTrenchNearX + margin;
        return besideSideWall && (inBlueTrenchX || inRedTrenchX);
    }

    /** @return true if any part of the robot (reach = center to farthest edge, plus margin) is in a trench now
     *  or will be within lookaheadSeconds at its current field-relative speed */
    public static boolean robotHeadingIntoTrench(Pose2d pose, ChassisSpeeds fieldSpeeds, double reach, double lookaheadSeconds) {
        for (double t = 0.0; t <= lookaheadSeconds + 1e-9; t += lookaheadSeconds / 4) {
            Translation2d future = pose.getTranslation().plus(new Translation2d(
                fieldSpeeds.vxMetersPerSecond * t, fieldSpeeds.vyMetersPerSecond * t));
            if (isNearTrench(future, reach)) {
                return true;
            }
        }
        return false;
    }
}
