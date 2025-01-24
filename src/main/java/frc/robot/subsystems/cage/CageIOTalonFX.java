package frc.robot.subsystems.cage;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.cage.CageIO.CageIOInputs;


public class CageIOTalonFX implements CageIO {
    private TalonFX motor = new TalonFX(0);

    public CageIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(CageIOInputs inputs) {
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
        // TODO the following rotational values need to be in meters and m/s
        inputs.position = motor.getPosition().getValueAsDouble();
    }

    @Override
    public void setPosition(double targetPosition) {
        motor.setControl(new PositionVoltage(targetPosition));
    }
}
