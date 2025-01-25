package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static final double gearing = 3.0;
    public static final double motorToElevatorPosition = 1.0; // TODO figure this out when testing
    public static final double kPDefault = 0.0;
    public static final double kDDefault = 0.0;

    public static final double stage1Range = Units.inchesToMeters(24.250000);
    public static final double stage2Range = Units.inchesToMeters(24.000000);
    public static final double stage3Range = Units.inchesToMeters(19.500000);

    public static final double stationPickupHeight = 0.4;
    public static final double l1ScoreHeight = 0.1;
    public static final double l2ScoreHeight = 0.3;
    public static final double l3ScoreHeight = 0.6;
    public static final double l4ScoreHeight = 1.0;
}
