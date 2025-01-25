package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
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

    public double getExtensionMeters() {
        return inputs.elevatorPositionMeters;
    }

    public Command setVoltage(DoubleSupplier voltageSupplier) {
        return run(() -> io.setVolts(voltageSupplier.getAsDouble()));
    }
}
