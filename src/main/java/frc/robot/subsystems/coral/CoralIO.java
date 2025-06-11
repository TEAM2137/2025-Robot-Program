package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
    @AutoLog
    public static class CoralIOInputs{
        public double velocityRadPerSec = 0.0;
        public double targetVelocityRadPerSec = 0.0;

        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        public double endEffectorDistanceCm = 0.0;
        public double funnelDistanceCm = 0.0;
        public double coralDistanceCm = 0.0;

        public boolean funnelConnected = false;
        public boolean endEffectorConnected = false;
        public boolean coralSensorConnected = false;
    }

    public default void updateInputs(CoralIOInputs inputs) {}

    public default void setRollerVoltage(double voltage) {}

    public default void setRollerVelocity(double velocityRadPerSec) {}
}
