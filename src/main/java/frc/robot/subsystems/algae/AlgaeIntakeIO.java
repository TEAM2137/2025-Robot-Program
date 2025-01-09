package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
    @AutoLog
    public class AlgaeIntakeIOInputs {
        public double pivotPosition = 0.0;
        public double pivotVelocity = 0.0;
        public double pivotVoltage = 0.0;
        public double pitvotAmps = 0.0;

        public double rollerVoltage = 0.0;
        public double rollerAmps = 0.0;
    }

    public default void updateInputs(AlgaeIntakeIOInputs inputs) {}

    public default void setPivotPosition(double position) {}

    public default void setRollerVoltage(double voltage) {}
}
