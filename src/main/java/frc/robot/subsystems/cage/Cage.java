package frc.robot.subsystems.cage;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class Cage extends SubsystemBase{
    private final CageIO io;
    private final CageIOInputsAutoLogged inputs;

    private boolean shouldZeroOnEnable = true;

    public Cage(CageIO io){
        this.io = io;
        this.inputs = new CageIOInputsAutoLogged();

        // Reset elevator position when enabled for the first time
        RobotModeTriggers.disabled().negate().and(() -> this.shouldZeroOnEnable)
                .onTrue(resetPositionCommand().andThen(() -> this.shouldZeroOnEnable = false));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Cage", inputs);
    }

    public Command setPosition(double targetPosition) {
        return runOnce(() -> io.setPosition(targetPosition));
    }

    public Command setVoltage(DoubleSupplier voltageSupplier) {
        return run(() -> io.setVoltage(voltageSupplier.getAsDouble()));
    }

    public Command resetPositionCommand() {
        return runOnce(() -> io.resetPosition());
    }
}
