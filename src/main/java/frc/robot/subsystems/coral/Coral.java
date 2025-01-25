package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral.CoralConstants.ConveyorConstants;

public class Coral extends SubsystemBase{
    private DigitalInput initialConveyorSensor;
    private final CoralIO io;
    private final CoralIOInputsAutoLogged inputs;

    public Coral(CoralIO io){
        this.io = io;
        this.inputs = new CoralIOInputsAutoLogged();
    }
    public void Conveyor() {
        initialConveyorSensor = new DigitalInput(ConveyorConstants.initialConveyorSensor);
    }

    public boolean getInitialConveyorSensor() {
        return !initialConveyorSensor.get();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral", inputs);

    }
    public Command setRollerVoltage(double voltage){
        return runOnce( () -> {
            io.setRollerVoltage(voltage);
        });
    }

    public Command changeStatus(boolean converorSwitchStatus){
        return runOnce( () -> {
            io.updateStatus(converorSwitchStatus, inputs);
        });
    }
}
