package frc.robot.subsystems.cage;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class CageIOSparkMax implements CageIO {
    private SparkMax motor;
    private PIDController controller = new PIDController(CageConstants.kP, 0, CageConstants.kD);
    private boolean usePID = false;

    public CageIOSparkMax() {
        motor = new SparkMax(CageConstants.motorID, MotorType.kBrushless);

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
        if (usePID) {
            double volts = controller.calculate(motor.getEncoder().getPosition());
            motor.getClosedLoopController().setReference(volts, ControlType.kVoltage);
        }

        inputs.appliedVolts = motor.getAppliedOutput() * 12;
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
        inputs.motorRotations = motor.getEncoder().getPosition();
        inputs.armAngle = Rotation2d.fromRotations(inputs.motorRotations * CageConstants.motorPositionToArmAngle);
    }

    @Override
    public void setPosition(double targetPosition) {
        controller.setSetpoint(targetPosition);
        this.usePID = true;
    }

    @Override
    public void setVoltage(double volts) {
        motor.getClosedLoopController().setReference(volts, ControlType.kVoltage);
        this.usePID = false;
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(0);
        this.usePID = false;
    }
}
