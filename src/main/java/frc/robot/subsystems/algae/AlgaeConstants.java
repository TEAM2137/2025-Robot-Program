package frc.robot.subsystems.algae;

public class AlgaeConstants {
    public static final int deviceID = 40;
    public static final double gearing = 16.0 * (48.0 / 18.0);

    // PID Constants
    public static final double kP = 5.0; // PID P
    public static final double kD = 0.01; // PID D
    public static final double targetVelocity = 42.0; // MM Velocity
    public static final double targetAccel = 65.0; // MM Acceleration

    // Setpoints
    public static final double deploy = -15.79;
    public static final double stow = 0.0;
}
