package frc.robot.autoalign;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

public record NearestTargetSelector(List<Pose2d> pool, double joystickInfluence) implements TargetSelector {
    public NearestTargetSelector(List<Pose2d> pool) {
        this(pool, 0.0);
    }

    @Override
    public Pose2d getPose(Context context) {
        Pair<Pose2d, Double> bestResult = new Pair<>(new Pose2d(), 1000.0);
        Pose2d robotPose = context.robotPose();

        for (Pose2d rawPose : pool) {
            // Calculate the distance from the robot to the current reef pole
            Pose2d pose = TargetSelector.flipIfRed(rawPose);
            double dst = robotPose.getTranslation().getDistance(pose.getTranslation());

            // Calculate additional weighting based on joystick angle
            double addition = LegacyAutoAlign.calculateBestPoseAddition(
                    pose.minus(robotPose).getTranslation(), context.joystickVector());

            // Apply addition and assign new best result if applicable
            double weight = dst + addition * joystickInfluence;
            if (weight <= bestResult.getSecond()) bestResult = new Pair<>(pose, weight);
        }

        return bestResult.getFirst();
    }
}
