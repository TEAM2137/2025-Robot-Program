package frc.robot.subsystems.cage;

import org.littletonrobotics.junction.AutoLog;

public interface CageIO {
    @AutoLog
    public static class CageIOInputs {
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double position = 0.0;
        public double cagearmPosition;
        public double cagearmVelocity;
        public double cagearmVoltage;
        public double cagearmAmps;
    }

    public default void updateInputs(CageIOInputs inputs) {}

    public default void setPosition(double position) {}
}
