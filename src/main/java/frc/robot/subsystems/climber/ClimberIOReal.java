package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

public class ClimberIOReal implements ClimberIO {
    private TalonFX rollersMotor;
    private SparkMax pivotMotor;

    private PIDController controller = new PIDController(ClimberConstants.kP, 0, ClimberConstants.kD);
    private boolean usePID = false;

    public ClimberIOReal() {
        rollersMotor = new TalonFX(ClimberConstants.rollersID);
        pivotMotor = new SparkMax(ClimberConstants.pivotID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false).idleMode(IdleMode.kBrake);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(ClimberConstants.kP)
            .d(ClimberConstants.kD);
        config.encoder
            .positionConversionFactor(1.0 / ClimberConstants.gearing)
            .velocityConversionFactor(1.0 / ClimberConstants.gearing);

        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rollersMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        if (usePID) {
            double volts = Math.min(Math.max(controller.calculate(pivotMotor.getEncoder().getPosition()), -12), 12);
            if (controller.getSetpoint() == ClimberConstants.climbPosition) volts *= 0.8;
            pivotMotor.getClosedLoopController().setReference(volts, ControlType.kVoltage);
        }

        inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * 12;
        inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();
        inputs.pivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotMotor.getEncoder().getVelocity());
        inputs.pivotPositionRotations = pivotMotor.getEncoder().getPosition();
        inputs.pivotAngle = Rotation2d.fromRotations(inputs.pivotPositionRotations * ClimberConstants.motorPositionToArmAngle);

        inputs.rollersAppliedVolts = rollersMotor.getMotorVoltage().getValueAsDouble();
        inputs.rollersCurrentAmps = rollersMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rollersVelocityRadPerSec = rollersMotor.getVelocity().getValue().in(RadiansPerSecond);
    }

    @Override
    public void setPivotPosition(double targetPosition) {
        controller.setSetpoint(targetPosition);
        this.usePID = true;
    }

    @Override
    public void setPivotVoltage(double volts) {
        pivotMotor.getClosedLoopController().setReference(volts, ControlType.kVoltage);
        this.usePID = false;
    }

    @Override
    public void setRollersVoltage(double volts) {
        rollersMotor.setVoltage(volts);
    }

    @Override
    public void resetPosition() {
        pivotMotor.getEncoder().setPosition(0);
        this.usePID = false;
    }
}
