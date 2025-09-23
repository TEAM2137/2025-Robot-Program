package frc.robot.autoalign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.FieldPOIs;
import org.littletonrobotics.junction.Logger;

import java.util.List;

/**
 * selects the first pose from the pool whose reef "face" matches the robot's reef face
 * @param pool
 */
public record ReefFaceTargetSelector(List<Pose2d> pool) implements TargetSelector {
    @Override
    public Pose2d getPose(Context context) {
        int robotFace = getReefFace(context.robotPose());
        for (Pose2d rawPose : pool) {
            Pose2d pose = TargetSelector.flipIfRed(rawPose);
            int targetFace = getReefFace(pose);
            if (targetFace == robotFace) return pose;
        }
        return new Pose2d();
    }

    public static int getReefFace(Pose2d pose) {
        Translation2d center = TargetSelector.flipIfRed(FieldPOIs.REEF_CENTER).getTranslation();
        Translation2d robot = pose.getTranslation();

        // calculate the angle between the reef and the robot
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(center.getY() - robot.getY(), center.getX() - robot.getX()));
        double rads = angle.getRadians() + Math.PI + (Math.PI / 6);

        // divide the angle by π/3 to find the reef face
        int face = ((int) (rads / (Math.PI / 3))) % 6;
        Logger.recordOutput("AutoAlign/ReefFace", face);
        return face;
    }
}
