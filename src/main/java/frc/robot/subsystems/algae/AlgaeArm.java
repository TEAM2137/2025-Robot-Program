package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class AlgaeArm extends SubsystemBase {
    public final AlgaeArmIO io;
    public final AlgaeIntakeIOInputsAutoLogged inputs;

    private boolean shouldZeroOnEnable = true;

    public AlgaeArm(AlgaeArmIO io) {
        this.io = io;
        this.inputs = new AlgaeIntakeIOInputsAutoLogged();

        // Reset elevator position when enabled for the first time
        RobotModeTriggers.disabled().negate().and(() -> this.shouldZeroOnEnable)
                .onTrue(resetPositionCommand().andThen(() -> this.shouldZeroOnEnable = false));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Algae", inputs);
    }

    public Command setPivotPosition(double position) {
        return runOnce(() -> io.setPivotPosition(position));
    }

    public Command resetPositionCommand() {
        return runOnce(() -> io.resetPosition());
    }
}
