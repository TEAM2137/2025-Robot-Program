package frc.robot.subsystems.coral;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Coral extends SubsystemBase{
    private final CoralIO io;
    private final CoralIOInputsAutoLogged inputs;

    public final Trigger beamBroken;
    private boolean isBeamBroken;

    public Coral(CoralIO io){
        this.io = io;
        this.inputs = new CoralIOInputsAutoLogged();
        this.beamBroken = new Trigger(() -> isBeamBroken);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        isBeamBroken = inputs.isBeamBroken;
        Logger.processInputs("Coral", inputs);
    }

    public Command setVoltageCommand(double voltage){
        return runOnce(() -> io.setRollerVoltage(voltage));
    }

    public Command setVoltageCommand(DoubleSupplier voltage){
        return runOnce(() -> io.setRollerVoltage(voltage.getAsDouble()));
    }

    public Command intakeCommand() {
        return intakeUntilBrokenCommand()
            .andThen(intakeWhileBrokenCommand())
            .andThen(setVoltageCommand(0));
    }

    public Command intakeUntilBrokenCommand() {
        return setVoltageCommand(2).repeatedly().until(beamBroken)
            .finallyDo(() -> io.setRollerVoltage(0));
    }

    public Command intakeWhileBrokenCommand() {
        return setVoltageCommand(2).repeatedly().onlyWhile(beamBroken)
            .finallyDo(() -> io.setRollerVoltage(0));
    }
}
