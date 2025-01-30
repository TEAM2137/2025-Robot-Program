package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    // IDs
    public static final int leaderID = 30;
    public static final int followerID = 31;

    // Ratios
    public static final double gearing = 3.0;
    public static final double spoolDiameter = Units.inchesToMeters(1.375 + 0.125);

    // PID Constants
    public static final double kS = 0.0; // Output addition to overcome static friction
    public static final double kG = 6.0; // Output addition to overcome gravity
    public static final double kP = 1.0; // PID P
    public static final double kD = 0.0; // PID D

    // CTRE Motion Magic Constants (https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html)
    public static final double targetCruiseVelocity = 4; // Target cruise velocity (rps)
    public static final double targetAcceleration = 1.5; // Target acceleration (rps/s)
    public static final double targetJerk = 5; // Target jerk (rps/s/s)

    // Ranges (m)
    public static final double stage1Range = Units.inchesToMeters(24.250000);
    public static final double stage2Range = Units.inchesToMeters(24.000000);
    public static final double stage3Range = Units.inchesToMeters(19.500000);

    // Setpoints (m)
    public static final double coralStationSetpoint = 0.1;
    public static final double l1Setpoint = 0.1;
    public static final double l2Setpoint = 0.445;
    public static final double l3Setpoint = 0.875;
    public static final double l4Setpoint = 1.45;
}
