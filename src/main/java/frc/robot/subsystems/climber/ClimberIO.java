package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotVelocityRadPerSec = 0.0;
        public double pivotPositionRotations = 0.0;
        public Rotation2d pivotAngle = new Rotation2d();

        public double rollersAppliedVolts = 0.0;
        public double rollersCurrentAmps = 0.0;
        public double rollersVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setPivotPosition(double position) {}
    public default void setPivotVoltage(double volts) {}

    public default void setRollersVoltage(double volts) {}

    public default void resetPosition() {}
}
