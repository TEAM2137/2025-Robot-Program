package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
    public default void updateInputs(CoralIOInputs inputs){}

    public default void setRollerVoltage(double voltage){}
    @AutoLog
    public static class CoralIOInputs{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

    }

}
