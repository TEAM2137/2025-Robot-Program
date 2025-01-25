package frc.robot.subsystems.cage;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class CageIOTalonFX implements CageIO {
    private TalonFX motor = new TalonFX(0);

    public CageIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = CageConstants.kP;
        config.Slot0.kD = CageConstants.kD;
        config.Feedback.RotorToSensorRatio = CageConstants.gearing;
        config.Feedback.FeedbackRemoteSensorID = CageConstants.encoderID;
        config.Feedback.FeedbackRotorOffset = -CageConstants.encoderZeroPosition;

        motor.getConfigurator().apply(config);
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(CageIOInputs inputs) {
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
        inputs.motorPositionRotations = motor.getPosition().getValueAsDouble();
        inputs.armAngle = Rotation2d.fromRotations(motor.getPosition().getValueAsDouble() * CageConstants.motorPositionToArmAngle);
    }

    @Override
    public void setPosition(double targetPosition) {
        motor.setControl(new PositionVoltage(targetPosition));
    }
}
