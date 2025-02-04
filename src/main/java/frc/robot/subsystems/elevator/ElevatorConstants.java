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
    public static final double kP = 4.5; // PID P
    public static final double kD = 0.1; // PID D

    // CTRE Motion Magic Constants (https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html)
    public static final double targetCruiseVelocity = 65.0; // Target cruise velocity (rps)
    public static final double targetAcceleration = 175.0; // Target acceleration (rps/s)

    // Ranges (m)
    public static final double stage1Range = Units.inchesToMeters(24.250000);
    public static final double stage2Range = Units.inchesToMeters(24.000000);
    public static final double stage3Range = Units.inchesToMeters(19.500000);

    // Setpoints (m)
    public static final double coralStationSetpoint = 0.0;
    public static final double l1Setpoint = 0.14;
    public static final double l2Setpoint = 0.38;
    public static final double l3Setpoint = 1.00;
    public static final double l4Setpoint = 1.82;
}
