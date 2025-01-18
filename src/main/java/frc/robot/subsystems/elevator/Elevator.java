package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public static class Constants {
        public static double CORAL_STATION = 0.4;
        public static double L1 = 0.1;
        public static double L2 = 0.3;
        public static double L3 = 0.6;
        public static double L4 = 1.0;
    }

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.inputs = new ElevatorIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public Command resetPositionCommand() {
        return runOnce(() -> {
            io.setTargetPosition(0);
        });
    }

    public Command setPositionCommand(double targetPosition) {
        return runOnce(() -> {
            io.setTargetPosition(targetPosition);
        });
    }
}
