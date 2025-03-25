package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FieldPOIs {
    private static final double REEF_VERTICAL_OFFSET_METERS = 1.235;
    private static final double ALGAE_ALIGN_VERTICAL_OFFSET_METERS = 0.25;
    private static final double ALGAE_GRAB_VERTICAL_OFFSET_METERS = 0.02;
    private static final double REEF_HORIZONTAL_OFFSET_METERS = 0.17;
    private static final double END_EFFECTOR_OFFSET = Units.inchesToMeters(-2.3);

    public static final double REEF_ZONE_DISTANCE = 1.75;

    public static final Pose2d REEF_CENTER = new Pose2d(new Translation2d(4.49, 4.03), new Rotation2d());
    public static final List<Pose2d> REEF_LOCATIONS = createReefLocations();

    public static final List<Pose2d> REEF_BRANCHES_RIGHT = filterEach(4, 0, REEF_LOCATIONS);
    public static final List<Pose2d> REEF_BRANCHES_LEFT = filterEach(4, 1, REEF_LOCATIONS);

    public static final List<Pose2d> ALGAE_ALIGN_LOCATIONS = filterEach(4, 2, REEF_LOCATIONS);
    public static final List<Pose2d> ALGAE_GRAB_LOCATIONS = filterEach(4, 3, REEF_LOCATIONS);

    public static final Pose2d NET = new Pose2d(new Translation2d(7.42, 0.0), new Rotation2d(0.0));

    public static final Pose2d CORAL_STATION_TOP = new Pose2d(new Translation2d(1.0, 7.107), new Rotation2d(-0.939));
    public static final Pose2d CORAL_STATION_BOTTOM = new Pose2d(new Translation2d(1.0, 0.918), new Rotation2d(0.939));

    static {
        // Publish values to NT
        NetworkTableInstance.getDefault()
                .getStructTopic("ReefCenter", Pose2d.struct)
                .publish().accept(REEF_CENTER);
        NetworkTableInstance.getDefault()
                .getStructArrayTopic("ReefLocations", Pose2d.struct)
                .publish().accept(REEF_LOCATIONS.toArray(new Pose2d[0]));
    }

    private static List<Pose2d> createReefLocations() {
        ArrayList<Pose2d> builder = new ArrayList<>(12);

        Rotation2d angle = Rotation2d.fromRadians(0);
        for (int i = 0; i < 6; i++) {
            angle = angle.plus(Rotation2d.fromRadians(Math.PI / 3));

            Translation2d verticalOffset = new Translation2d(
                angle.getCos() * (REEF_VERTICAL_OFFSET_METERS),
                angle.getSin() * (REEF_VERTICAL_OFFSET_METERS)
            );

            Translation2d algaeAlignVerticalOffset = new Translation2d(
                angle.getCos() * (REEF_VERTICAL_OFFSET_METERS + ALGAE_ALIGN_VERTICAL_OFFSET_METERS),
                angle.getSin() * (REEF_VERTICAL_OFFSET_METERS + ALGAE_ALIGN_VERTICAL_OFFSET_METERS)
            );

            Translation2d algaeGrabVerticalOffset = new Translation2d(
                angle.getCos() * (REEF_VERTICAL_OFFSET_METERS + ALGAE_GRAB_VERTICAL_OFFSET_METERS),
                angle.getSin() * (REEF_VERTICAL_OFFSET_METERS + ALGAE_GRAB_VERTICAL_OFFSET_METERS)
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

            Pose2d rightPole = new Pose2d(REEF_CENTER.getTranslation().plus(verticalOffset).plus(rightOffset), angle.plus(Rotation2d.k180deg));
            Pose2d leftPole = new Pose2d(REEF_CENTER.getTranslation().plus(verticalOffset).plus(leftOffset), angle.plus(Rotation2d.k180deg));
            Pose2d algaeStage1 = new Pose2d(REEF_CENTER.getTranslation().plus(algaeAlignVerticalOffset).plus(centerOffset), angle.plus(Rotation2d.k180deg));
            Pose2d algaeStage2 = new Pose2d(REEF_CENTER.getTranslation().plus(algaeGrabVerticalOffset).plus(centerOffset), angle.plus(Rotation2d.k180deg));

            builder.add(rightPole);
            builder.add(leftPole);
            builder.add(algaeStage1);
            builder.add(algaeStage2);
        }

        return List.copyOf(builder);
    }

    private static <T> List<T> filterEach(int groupSize, int offset, List<T> original) {
        ArrayList<T> builder = new ArrayList<>();
        for (int i = offset; i < original.size(); i += groupSize) builder.add(original.get(i));
        return List.copyOf(builder);
    }
}
