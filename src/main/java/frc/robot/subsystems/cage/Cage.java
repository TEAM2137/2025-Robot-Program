package frc.robot.subsystems.cage;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cage extends SubsystemBase{
    private final CageIO io;
    private final CageIOInputsAutoLogged inputs;

    public Cage(CageIO io){
        this.io = io;
        this.inputs = new CageIOInputsAutoLogged();
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
}
