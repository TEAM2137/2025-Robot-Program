package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;

    public final Trigger isUnused;

    // This is to make the elevator position only reset the first time it's enabled
    private boolean shouldZeroOnEnable = true;

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.io.resetPosition();
        this.inputs = new ElevatorIOInputsAutoLogged();

        // Reset elevator position when enabled for the first time
        RobotModeTriggers.disabled().negate().and(() -> this.shouldZeroOnEnable)
                .onTrue(resetPositionCommand().andThen(() -> this.shouldZeroOnEnable = false));

        this.isUnused = new Trigger(() -> getCurrentCommand() == null);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public Command resetPositionCommand() {
        return runOnce(() -> io.resetPosition());
    }

    public Command applyScheduledPositionCommand() {
        return runOnce(() -> io.applyScheduledPosition());
    }

    public void applyScheduledPosition() {
        io.applyScheduledPosition();
    }

    public Command schedulePositionCommand(double targetPosition) {
        return runOnce(() -> io.schedulePosition(targetPosition));
    }

    public void schedulePosition(double targetPosition) {
        io.schedulePosition(targetPosition);
    }

    public void setPosition(double targetPosition) {
        io.setTargetPosition(targetPosition);
    }

    public Command setPositionCommand(DoubleSupplier targetPosition) {
        return runOnce(() -> io.setTargetPosition(targetPosition.getAsDouble()));
    }

    public Command setPositionCommand(double targetPosition) {
        return runOnce(() -> io.setTargetPosition(targetPosition));
    }

    public double getExtensionMeters() {
        return inputs.elevatorPositionMeters;
    }

    public double getTargetPosition() {
        return inputs.targetPositionMeters;
    }

    public double getScheduledPosition() {
        return io.getScheduledPosition();
    }

    public Command setVoltage(DoubleSupplier voltageSupplier) {
        return run(() -> io.setVolts(voltageSupplier.getAsDouble()));
    }

    public Command stowCommand() {
        return schedulePositionCommand(ElevatorConstants.stow);
    }

    public boolean isAtTarget() {
        return Math.abs(getTargetPosition() - getExtensionMeters()) < 0.2;
    }
}
