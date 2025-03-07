package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    // IDs
    public static final int leaderID = 30;
    public static final int followerID = 31;

    // Ratios
    public static final double gearing = 37.0 / 12.0;
    public static final double spoolRadius = Units.inchesToMeters(1.25) / 2.0;

    // PID Constants
    public static final double kS = 0.0; // Output addition to overcome static friction
    public static final double kG = 0.05; // Output addition to overcome gravity
    public static final double kP = 5.5; // PID P
    public static final double kD = 0.13; // PID D

    // CTRE Motion Magic Constants (https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html)
    public static final double targetCruiseVelocity = 65.0; // Target cruise velocity (rps)
    public static final double targetAcceleration = 175.0; // Target acceleration (rps/s)

    // Ranges (m)
    public static final double stage1Range = Units.inchesToMeters(24.250000);
    public static final double stage2Range = Units.inchesToMeters(24.000000);
    public static final double stage3Range = Units.inchesToMeters(19.500000);

    // Setpoints (m)

    public static final double stow = -0.1;
    public static final double L1 = 0.05;
    public static final double L2 = 0.71;
    public static final double L3 = 1.22;
    public static final double L4 = 1.97;

    public static final double algaeHigh = L3;
    public static final double algaeLow = L2;
}
