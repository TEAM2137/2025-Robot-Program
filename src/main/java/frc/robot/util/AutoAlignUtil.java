package frc.robot.util;

import java.util.List;
import java.util.Map;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;

public class AutoAlignUtil {
    /**
     * Target types for auto align
     */
    public enum Target {
        LEFT_POLE,
        RIGHT_POLE,
        ALGAE
    }

    /**
     * A key-value map storing the pose data for a given auto align target type
     */
    private static final Map<Target, List<Pose2d>> targetToPoseData = Map.of(
        // Left reef poles
        Target.LEFT_POLE, FieldPOIs.REEF_LOCATIONS_LEFT,

        // Right reef poles
        Target.RIGHT_POLE, FieldPOIs.REEF_LOCATIONS_RIGHT,

        // Locations for removing algae
        Target.ALGAE, FieldPOIs.ALGAE_LOCATIONS
    );

    /**
     * Retrieves the nearest integer pose ID for a given auto align target type
     */
    public static int mapToPoseId(Target targetType, Drive drive, Translation2d motionVector) {
        List<Pose2d> poseData = targetToPoseData.get(targetType);
        return drive.getNearestPose(drive.getPose(), motionVector, poseData);
    }

    /**
     * Retrieves the nearest pose for a given auto align target type
     */
    public static Pose2d mapToPose(Target targetType, Drive drive, Translation2d motionVector) {
        List<Pose2d> poseData = targetToPoseData.get(targetType);
        return poseData.get(drive.getNearestPose(drive.getPose(), motionVector, poseData));
    }

    public static Pose2d fromPoseId(int id, Target targetType) {
        List<Pose2d> poseData = targetToPoseData.get(targetType);
        return poseData.get(id);
    }

    /**
     * Calculates a value to add to the selection "weight" of each reef pole.
     * This is determined by the dot product of the robot to reef pole vector and the vector of the
     * joystick motion. This is to ensure that the robot will prefer to target reef faces that the
     * driver is moving towards.
     */
    public static double calculateBestReefPoleAddition(Translation2d toReefVector, Translation2d motionVector) {
        if (motionVector.getNorm() < 0.1) return 0.0;
        return dot(normalize(toReefVector), normalize(motionVector));
    }

    /**
     * @return A copy of the given translation vector with a magnitude of 1
     */
    public static Translation2d normalize(Translation2d vector) {
        return vector.div(vector.getNorm());
    }

    /**
     * @return The dot product of translations a and b
     */
    public static double dot(Translation2d a, Translation2d b) {
        return (a.getX() * b.getX() + a.getY() * b.getY());
    }

    /**
     * Uses choreo utility methods to flip the given pose if on red alliance
     */
    public static Pose2d flipIfRed(Pose2d pose) {
        return ChoreoAllianceFlipUtil.shouldFlip() ? ChoreoAllianceFlipUtil.flip(pose) : pose;
    }
}
