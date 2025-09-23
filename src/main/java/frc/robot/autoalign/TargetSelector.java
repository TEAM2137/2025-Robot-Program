package frc.robot.autoalign;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.FieldPOIs;

import java.util.List;

/**
 * This interface handles the selection of target poses during the auto align process.
 * Inheritors must choose what pose to output given a {@code TargetSelector.Context} instance.
 *
 * @author Avery Gardner
 * @since 2025
 */
public interface TargetSelector {
    TargetSelector LEFT_BRANCHES = new ReefFaceTargetSelector(FieldPOIs.REEF_BRANCHES_LEFT);
    TargetSelector RIGHT_BRANCHES = new ReefFaceTargetSelector(FieldPOIs.REEF_BRANCHES_RIGHT);
    TargetSelector ALGAE_ALIGN = new ReefFaceTargetSelector(FieldPOIs.ALGAE_ALIGN_LOCATIONS);
    TargetSelector ALGAE_GRAB = new ReefFaceTargetSelector(FieldPOIs.ALGAE_GRAB_LOCATIONS);
    TargetSelector NET = new NearestTargetSelector(List.of(FieldPOIs.NET, FieldPOIs.NET_OPPOSITE));

    /**
     * @return the pose to target for the given context of the situation
     */
    Pose2d getPose(Context context);

    /**
     * @return true if this target selector allows {@code getPose()} to be called each loop cycle
     */
    default boolean isDynamic() { return false; }

    /**
     * Uses choreo utility methods to flip the given pose if on red alliance
     */
    static Pose2d flipIfRed(Pose2d pose) {
        return ChoreoAllianceFlipUtil.shouldFlip() ? ChoreoAllianceFlipUtil.flip(pose) : pose;
    }

    record Context(Pose2d robotPose, Translation2d joystickVector) {}
}
