package frc.robot.subsystems.cage;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class CageIOSparkMax implements CageIO {
    private SparkMax motor;
    private SparkClosedLoopController controller;

    public CageIOSparkMax() {
        motor = new SparkMax(CageConstants.motorID, MotorType.kBrushless);
        controller = motor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(CageConstants.kP)
            .d(CageConstants.kD);
        config.encoder
            .positionConversionFactor(1.0 / CageConstants.gearing)
            .velocityConversionFactor(1.0 / CageConstants.gearing);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(CageIOInputs inputs) {
        inputs.appliedVolts = motor.getAppliedOutput() * 12;
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(motor.getAbsoluteEncoder().getVelocity());
        inputs.motorRotations = motor.getAbsoluteEncoder().getPosition();
        inputs.armAngle = Rotation2d.fromRotations(inputs.motorRotations * CageConstants.motorPositionToArmAngle);
    }

    @Override
    public void setPosition(double targetPosition) {
        controller.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
    }

    @Override
    public void setVoltage(double volts) {
        controller.setReference(volts, ControlType.kVoltage);
    }
}
