package frc.robot.subsystems.coral;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Coral extends SubsystemBase{
    private final CoralIO io;
    private final CoralIOInputsAutoLogged inputs;

    public final Trigger endEffectorSensor;
    public final Trigger funnelSensor;
    private boolean isEndEffectorSensorInRange;
    private boolean isFunnelSensorInRange;

    public Coral(CoralIO io){
        this.io = io;
        this.inputs = new CoralIOInputsAutoLogged();
        this.endEffectorSensor = new Trigger(() -> isEndEffectorSensorInRange);
        this.funnelSensor = new Trigger(() -> isFunnelSensorInRange);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        isEndEffectorSensorInRange = inputs.endEffectorDistanceCm < CoralConstants.sensorRange;
        isFunnelSensorInRange = inputs.funnelDistanceCm < CoralConstants.funnelSensorRange;
        Logger.processInputs("Coral", inputs);
    }

    public Command setVoltageCommand(double voltage){
        return runOnce(() -> io.setRollerVoltage(voltage));
    }

    public Command setVoltageCommand(DoubleSupplier voltage){
        return runOnce(() -> io.setRollerVoltage(voltage.getAsDouble()));
    }

    public Command setVelocityCommand(double velocityRadPerSec) {
        return runOnce(() -> io.setRollerVelocity(velocityRadPerSec));
    }

    public Command setVelocityCommand(DoubleSupplier velocityRadPerSec) {
        return runOnce(() -> io.setRollerVelocity(velocityRadPerSec.getAsDouble()));
    }

    public Command intakeCommand() {
        return intakeUntilFunnelEnter()
            .andThen(completeIntaking())
            .andThen(setVoltageCommand(0));
    }

    public Command intakeUntilFunnelEnter() {
        return setVoltageCommand(5).repeatedly().until(funnelSensor.or(endEffectorSensor))
            .finallyDo(() -> io.setRollerVoltage(0));
    }

    public Command completeIntaking() {
        return setVoltageCommand(5).repeatedly().until(endEffectorSensor)
            .andThen(setVoltageCommand(2).repeatedly().onlyWhile(endEffectorSensor)
            .finallyDo(() -> io.setRollerVoltage(0)));
    }
}
