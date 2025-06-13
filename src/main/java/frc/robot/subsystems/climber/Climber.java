package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class Climber extends SubsystemBase{
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;

    private boolean shouldZeroOnEnable = true;

    public Climber(ClimberIO io){
        this.io = io;
        this.inputs = new ClimberIOInputsAutoLogged();

        // Reset elevator position when enabled for the first time
        RobotModeTriggers.disabled().negate().and(() -> this.shouldZeroOnEnable)
                .onTrue(resetPositionCommand().andThen(() -> this.shouldZeroOnEnable = false));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public Command setPositionCommand(double targetPosition) {
        return runOnce(() -> io.setPosition(targetPosition));
    }

    public Command setVoltage(DoubleSupplier voltageSupplier) {
        return run(() -> io.setVoltage(voltageSupplier.getAsDouble()));
    }

    public Command resetPositionCommand() {
        return runOnce(() -> io.resetPosition());
    }
}
