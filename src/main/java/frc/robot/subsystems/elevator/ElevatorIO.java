package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double motorPositionRotations = 0.0;
        public double elevatorPositionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;

        public double leaderOutputVolts = 0.0;
        public double leaderCurrentAmps = 0.0;

        public double followerOutputVolts = 0.0;
        public double followerCurrentAmps = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setTargetPosition(double targetPosition) {}

    public default void setVolts(double appliedVolts) {}

    public default void setPIDConstants(double kP, double kD) {}

    public default void setFFConstants(double kS, double kG) {}

    public default void setMMConstants(double v, double a, double j) {}

    public default void resetPosition() {}
}
