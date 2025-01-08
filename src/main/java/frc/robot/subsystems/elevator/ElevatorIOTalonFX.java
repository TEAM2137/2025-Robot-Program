package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX motor = new TalonFX(0);

    public ElevatorIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
        // TODO the following rotational values need to be in meters and m/s
        inputs.positionMeters = motor.getPosition().getValueAsDouble();
        inputs.velocityMetersPerSecond = motor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setTargetPosition(double targetPosition) {
        motor.setControl(new PositionVoltage(targetPosition));
    }

    @Override
    public void setVolts(double appliedVolts) {

    }
}
