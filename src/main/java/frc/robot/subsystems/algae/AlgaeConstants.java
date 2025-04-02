package frc.robot.subsystems.algae;

public class AlgaeConstants {
    public static final int deviceID = 40;
    public static final int encoderID = 41;
    public static final double encoderOffset = 0.7351;
    public static final double gearing = 4.0;

    // PID Constants
    public static final double kP = 10.0; // PID P
    public static final double kD = 0.0; // PID D
    public static final double targetVelocity = 8.0; // MM Velocity
    public static final double targetAccel = 15.0; // MM Acceleration

    // Setpoints
    public static final double grab = 0.55;
    public static final double hold = 0.2;
    public static final double stow = -0.01;
    public static final double intake = 0.06;
    public static final double processor = 0.72;
    public static final double groundIntake = 0.92;
}
