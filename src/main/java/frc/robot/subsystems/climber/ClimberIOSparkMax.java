package frc.robot.subsystems.climber;

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

public class ClimberIOSparkMax implements ClimberIO {
    private SparkMax motor;
    private PIDController controller = new PIDController(ClimberConstants.kP, 0, ClimberConstants.kD);
    private boolean usePID = false;

    public ClimberIOSparkMax() {
        motor = new SparkMax(ClimberConstants.motorID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(ClimberConstants.kP)
            .d(ClimberConstants.kD);
        config.encoder
            .positionConversionFactor(1.0 / ClimberConstants.gearing)
            .velocityConversionFactor(1.0 / ClimberConstants.gearing);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        if (usePID) {
            double volts = Math.min(Math.max(controller.calculate(motor.getEncoder().getPosition()), -12), 12);
            if (controller.getSetpoint() == ClimberConstants.climbPosition) volts *= 0.8;
            motor.getClosedLoopController().setReference(volts, ControlType.kVoltage);
        }

        inputs.appliedVolts = motor.getAppliedOutput() * 12;
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
        inputs.motorRotations = motor.getEncoder().getPosition();
        inputs.armAngle = Rotation2d.fromRotations(inputs.motorRotations * ClimberConstants.motorPositionToArmAngle);
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
