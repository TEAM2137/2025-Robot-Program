package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    public final AlgaeIntakeIO io;
    public final AlgaeIntakeIOInputsAutoLogged inputs;

    public AlgaeIntake(AlgaeIntakeIO io) {
        this.io = io;
        this.inputs = new AlgaeIntakeIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Algae", inputs);
    }

    public Command setPivotPosition(double position) {
        return runOnce(() -> {
            io.setPivotPosition(position);
        });
    }

    public Command setRollerVoltage(double voltage) {
        return runOnce(() -> {
            io.setRollerVoltage(voltage);
        });
    }
}
