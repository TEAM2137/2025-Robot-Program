package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CoralIOTalonFX implements CoralIO {
    public TalonFX rollers = new TalonFX(CoralConstants.rollersID, "rio");

    public CoralIOTalonFX() {
        // Create TalonFX config
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Set frequencies of used signals
        BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(100),
            rollers.getMotorVoltage(),
            rollers.getSupplyCurrent()
        );

        // Apply configurations
        rollers.getConfigurator().apply(config);
        rollers.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        inputs.appliedVolts = rollers.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = rollers.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollers.setControl(new VoltageOut(voltage));
    }
}
