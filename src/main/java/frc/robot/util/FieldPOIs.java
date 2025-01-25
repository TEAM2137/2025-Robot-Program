package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FieldPOIs {
    private static final double REEF_SCORING_OFFSET_METERS = 1.35;

    public static final Pose2d REEF = new Pose2d(new Translation2d(4.49, 4.03), new Rotation2d());
    public static final List<Pose2d> REEF_LOCATIONS = createReefScoringLocations();
    public static final List<Pose2d> REEF_LOCATIONS_RIGHT = filterEveryOther(1, REEF_LOCATIONS);
    public static final List<Pose2d> REEF_LOCATIONS_LEFT = filterEveryOther(0, REEF_LOCATIONS);

    static {
        // Publish values to NT
        NetworkTableInstance.getDefault()
                .getStructTopic("Reef", Pose2d.struct)
                .publish().accept(REEF);
        NetworkTableInstance.getDefault()
                .getStructArrayTopic("ReefLocations", Pose2d.struct)
                .publish().accept(REEF_LOCATIONS.toArray(new Pose2d[0]));
    }

    private static List<Pose2d> createReefScoringLocations() {
        ArrayList<Pose2d> builder = new ArrayList<>(12);

        Rotation2d angle = Rotation2d.fromRadians(0);
        for (int i = 0; i < 6; i++) {
            angle = angle.plus(Rotation2d.fromRadians(Math.PI / 3));

            Translation2d verticalOffset = new Translation2d(
                angle.getCos() * REEF_SCORING_OFFSET_METERS,
                angle.getSin() * REEF_SCORING_OFFSET_METERS
            );

            Translation2d leftOffset = new Translation2d(
                angle.minus(Rotation2d.kCW_90deg).getCos() * 0.17,
                angle.minus(Rotation2d.kCW_90deg).getSin() * 0.17
            );

            Translation2d rightOffset = new Translation2d(
                angle.plus(Rotation2d.kCW_90deg).getCos() * 0.17,
                angle.plus(Rotation2d.kCW_90deg).getSin() * 0.17
            );

            Pose2d leftPole = new Pose2d(REEF.getTranslation().plus(verticalOffset).plus(leftOffset), angle.plus(Rotation2d.k180deg));
            Pose2d rightPole = new Pose2d(REEF.getTranslation().plus(verticalOffset).plus(rightOffset), angle.plus(Rotation2d.k180deg));

            builder.add(leftPole);
            builder.add(rightPole);
        }

        return List.copyOf(builder);
    }

    private static <T> List<T> filterEveryOther(int offset, List<T> original) {
        ArrayList<T> builder = new ArrayList<>();
        for (int i = offset; i < original.size(); i += 2) builder.add(original.get(i));
        return List.copyOf(builder);
    }
}
