package frc.robot.subsystems.coral;

public class CoralConstants {
    public static final int rollersID = 32;
    public static final int endEffectorSensorID = 0; // TODO tune this
    public static final int funnelSensorID = 0; // TODO tune this

    // Sensor detection range (cm)
    public static final double sensorRange = 0; // TODO tune this

    // Roller Speeds
    public static final double l1RadPerSec = 171.5;
    public static final double scoreRadPerSec = 226.5;
    public static final double l4RadPerSec = 275.5;

    public static final double slowSpeed = 3.8;
    public static final double l1Speed = 3.0;
    public static final double l4Speed = 4.6;

    // PID Constants
    public static final double kP = 0.10; // PID P
    public static final double kS = 0.12; // FF Static
    public static final double kV = 0.101; // FF Velocity
}
