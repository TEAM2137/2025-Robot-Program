package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double velocityRadPerSec = 0.0;
        public double motorRotations = 0.0;
        public Rotation2d armAngle = new Rotation2d();
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setPosition(double position) {}

    public default void setVoltage(double volts) {}

    public default void resetPosition() {}
}
