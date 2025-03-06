package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double motorPositionRotations = 0.0;
        public double elevatorPositionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;

        public double appliedVolts = 0.0;
        public double scheduledTargetPosition = 0.0;
        public double targetPositionMeters = 0.0;
        public double targetPositionRotations = 0.0;

        public double leaderOutputVolts = 0.0;
        public double leaderCurrentAmps = 0.0;

        public double followerOutputVolts = 0.0;
        public double followerCurrentAmps = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void schedulePosition(double targetPosition) {}

    public default double getScheduledPosition() { return 0.0; }

    public default void applyScheduledPosition() {}

    /** Sets the setpoint for the elevator height, in meters */
    public default void setTargetPosition(double targetPosition) {}

    /** Sets the voltage of the elevator motors for manual control */
    public default void setVolts(double appliedVolts) {}

    /** Sets the kP and kD contants of the elevator control */
    public default void setPIDConstants(double kP, double kD) {}

    /** Sets the kS and kG contants of the elevator control */
    public default void setFFConstants(double kS, double kG) {}

    /** Sets the motion magic contants of the elevator control */
    public default void setMMConstants(double v, double a, double j) {}

    /** Resets the elevator's encoder position to 0 rotations/meters */
    public default void resetPosition() {}
}
