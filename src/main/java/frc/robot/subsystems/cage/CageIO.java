package frc.robot.subsystems.cage;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface CageIO {
    @AutoLog
    public static class CageIOInputs {
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double velocityRadPerSec = 0.0;
        public double motorPositionRotations = 0.0;
        public Rotation2d armAngle = new Rotation2d();
    }

    public default void updateInputs(CageIOInputs inputs) {}

    public default void setPosition(double position) {}
}
