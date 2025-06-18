package frc.robot.subsystems.algae;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AlgaeArm extends SubsystemBase {
    public final AlgaeArmIO io;
    public final AlgaeIntakeIOInputsAutoLogged inputs;

    public final Trigger isUnused;
    public final Trigger hasAlgae;

    private final Alert algaeSensorAlert = new Alert("Algae distance sensor disconnected. you're cooked", AlertType.kError);

    private boolean shouldZeroOnEnable = true;

    public AlgaeArm(AlgaeArmIO io) {
        this.io = io;
        this.inputs = new AlgaeIntakeIOInputsAutoLogged();
        this.hasAlgae = new Trigger(() -> inputs.algaeIsDetected).debounce(0.1);

        this.isUnused = new Trigger(() -> getCurrentCommand() == null);

        // Reset elevator position when enabled for the first time
        RobotModeTriggers.disabled().negate().and(() -> this.shouldZeroOnEnable)
                .onTrue(resetPositionCommand().andThen(() -> this.shouldZeroOnEnable = false));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        algaeSensorAlert.set(!inputs.algaeSensorConnected);

        Logger.processInputs("Algae", inputs);
    }

    public Command setPivotPosition(double position) {
        return runOnce(() -> io.setPivotPosition(position));
    }

    public Command setPivotVoltage(DoubleSupplier voltage) {
        return run(() -> io.setPivotVoltage(voltage.getAsDouble()));
    }

    public Command resetPositionCommand() {
        return runOnce(() -> io.resetPosition());
    }

    public boolean isHoldingAlgae() {
        return io.getTargetPosition() == AlgaeConstants.hold;
    }

    public Command targetCurrentPosition() {
        return runOnce(() -> io.setPivotPosition(io.getPivotPosition()));
    }

    public void setPivotPositionRaw(double position) {
        io.setPivotPosition(position);
    }
}
