package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FieldPOIs {
    private static final double REEF_VERTICAL_OFFSET_METERS = 1.26;
    private static final double ALGAE_VERTICAL_OFFSET_METERS = 0.14;
    private static final double REEF_HORIZONTAL_OFFSET_METERS = 0.17;
    private static final double END_EFFECTOR_OFFSET = Units.inchesToMeters(-2.3);

    public static final Pose2d REEF = new Pose2d(new Translation2d(4.49, 4.03), new Rotation2d());
    public static final List<Pose2d> REEF_BRANCHES = createReefScoringLocations();
    public static final List<Pose2d> REEF_BRANCHES_RIGHT = filterEveryThird(0, REEF_BRANCHES);
    public static final List<Pose2d> REEF_BRANCHES_LEFT = filterEveryThird(1, REEF_BRANCHES);
    public static final List<Pose2d> ALGAE_LOCATIONS = filterEveryThird(2, REEF_BRANCHES);

    public static final Pose2d NET = new Pose2d(new Translation2d(7.42, 0.0), new Rotation2d(0.0));

    public static final Pose2d CORAL_STATION_TOP = new Pose2d(new Translation2d(1.0, 7.107), new Rotation2d(-0.939));
    public static final Pose2d CORAL_STATION_BOTTOM = new Pose2d(new Translation2d(1.0, 0.918), new Rotation2d(0.939));

    static {
        // Publish values to NT
        NetworkTableInstance.getDefault()
                .getStructTopic("Reef", Pose2d.struct)
                .publish().accept(REEF);
        NetworkTableInstance.getDefault()
                .getStructArrayTopic("ReefLocations", Pose2d.struct)
                .publish().accept(REEF_BRANCHES.toArray(new Pose2d[0]));
    }

    private static List<Pose2d> createReefScoringLocations() {
        ArrayList<Pose2d> builder = new ArrayList<>(12);

        Rotation2d angle = Rotation2d.fromRadians(0);
        for (int i = 0; i < 6; i++) {
            angle = angle.plus(Rotation2d.fromRadians(Math.PI / 3));

            Translation2d verticalOffset = new Translation2d(
                angle.getCos() * (REEF_VERTICAL_OFFSET_METERS),
                angle.getSin() * (REEF_VERTICAL_OFFSET_METERS)
            );

            Translation2d algaeVerticalOffset = new Translation2d(
                angle.getCos() * (REEF_VERTICAL_OFFSET_METERS + ALGAE_VERTICAL_OFFSET_METERS),
                angle.getSin() * (REEF_VERTICAL_OFFSET_METERS + ALGAE_VERTICAL_OFFSET_METERS)
            );

            double leftMagnitude = REEF_HORIZONTAL_OFFSET_METERS - END_EFFECTOR_OFFSET;
            double centerMagnitude = -END_EFFECTOR_OFFSET;
            double rightMagnitude = REEF_HORIZONTAL_OFFSET_METERS + END_EFFECTOR_OFFSET;

            Translation2d centerOffset = new Translation2d(
                angle.minus(Rotation2d.kCW_90deg).getCos() * centerMagnitude,
                angle.minus(Rotation2d.kCW_90deg).getSin() * centerMagnitude
            );

            Translation2d rightOffset = new Translation2d(
                angle.minus(Rotation2d.kCW_90deg).getCos() * leftMagnitude,
                angle.minus(Rotation2d.kCW_90deg).getSin() * leftMagnitude
            );

            Translation2d leftOffset = new Translation2d(
                angle.plus(Rotation2d.kCW_90deg).getCos() * rightMagnitude,
                angle.plus(Rotation2d.kCW_90deg).getSin() * rightMagnitude
            );

            Pose2d rightPole = new Pose2d(REEF.getTranslation().plus(verticalOffset).plus(rightOffset), angle.plus(Rotation2d.k180deg));
            Pose2d leftPole = new Pose2d(REEF.getTranslation().plus(verticalOffset).plus(leftOffset), angle.plus(Rotation2d.k180deg));
            Pose2d algae = new Pose2d(REEF.getTranslation().plus(algaeVerticalOffset).plus(centerOffset), angle.plus(Rotation2d.k180deg));

            builder.add(rightPole);
            builder.add(leftPole);
            builder.add(algae);
        }

        return List.copyOf(builder);
    }

    private static <T> List<T> filterEveryThird(int offset, List<T> original) {
        ArrayList<T> builder = new ArrayList<>();
        for (int i = offset; i < original.size(); i += 3) builder.add(original.get(i));
        return List.copyOf(builder);
    }
}
