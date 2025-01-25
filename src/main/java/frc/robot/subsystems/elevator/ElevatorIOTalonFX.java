package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX leadmotor = new TalonFX(0);
    private TalonFX followmotor = new TalonFX(1);

    public ElevatorIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leadmotor.getConfigurator().apply(config);
        leadmotor.optimizeBusUtilization();
        followmotor.setControl(new Follower(leadmotor.getDeviceID(), true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = leadmotor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = leadmotor.getSupplyCurrent().getValueAsDouble();
        // TODO the following rotational values need to be in meters and m/s
        inputs.positionMeters = leadmotor.getPosition().getValueAsDouble();
        inputs.velocityMetersPerSecond = leadmotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setTargetPosition(double targetPosition) {
        leadmotor.setControl(new PositionVoltage(targetPosition));
    }

    @Override
    public void setVolts(double appliedVolts) {
        leadmotor.setControl(new VoltageOut(appliedVolts));
    }
}
