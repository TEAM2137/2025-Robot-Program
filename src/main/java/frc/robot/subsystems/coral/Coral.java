package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Coral extends SubsystemBase{
    private final CoralIO io;
    private final CoralIOInputsAutoLogged inputs;

    public final Trigger beamBrokenTrigger;

    public Coral(CoralIO io){
        this.io = io;
        this.inputs = new CoralIOInputsAutoLogged();
        this.beamBrokenTrigger = new Trigger(io::isBeamBroken);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral", inputs);
    }

    public Command setVoltageCommand(double voltage){
        return runOnce(() -> io.setRollerVoltage(voltage));
    }

    public Command intakeCommand() {
        return setVoltageCommand(2).repeatedly().until(beamBrokenTrigger)
            .andThen(setVoltageCommand(2).repeatedly().onlyWhile(beamBrokenTrigger))
            .andThen(setVoltageCommand(0));
    }

    public Command intakeUntilBrokenCommand(double timeoutSeconds) {
        return setVoltageCommand(2).repeatedly().until(beamBrokenTrigger)
            .withTimeout(timeoutSeconds)
            .finallyDo(() -> io.setRollerVoltage(0));
    }

    public Command intakeUntilNotBrokenCommand(double timeoutSeconds) {
        return setVoltageCommand(2).repeatedly().onlyWhile(beamBrokenTrigger)
            .withTimeout(timeoutSeconds)
            .finallyDo(() -> io.setRollerVoltage(0));
    }
}
