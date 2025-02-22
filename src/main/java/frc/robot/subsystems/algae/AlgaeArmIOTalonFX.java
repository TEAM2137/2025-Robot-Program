package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class AlgaeArmIOTalonFX implements AlgaeArmIO {
    public TalonFX pivotMotor = new TalonFX(AlgaeConstants.deviceID, "rio");

    public AlgaeArmIOTalonFX() {
        var config = new TalonFXConfiguration();

        // PID and FF settings
        var slot0Configs = config.Slot0;
        slot0Configs.kP = AlgaeConstants.kP;
        slot0Configs.kD = AlgaeConstants.kD;

        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = AlgaeConstants.targetVelocity;
        motionMagicConfigs.MotionMagicAcceleration = AlgaeConstants.targetAccel;

        // Motor feedback settings
        var feedbackConfigs = config.Feedback;
        feedbackConfigs.RotorToSensorRatio = AlgaeConstants.gearing;

        // Current limit settings
        var currentLimitsConfigs = config.CurrentLimits;
        currentLimitsConfigs.StatorCurrentLimit = 80;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        // Motor output settings
        var motorOutputConfig = config.MotorOutput;
        motorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
        motorOutputConfig.NeutralMode = NeutralModeValue.Brake;

        // Apply configurations
        pivotMotor.getConfigurator().apply(config);

        // Set frequencies of used signals
        BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(100),
            pivotMotor.getPosition(),
            pivotMotor.getVelocity(),
            pivotMotor.getMotorVoltage(),
            pivotMotor.getSupplyCurrent()
        );

        // Clear unused signals
        pivotMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        inputs.pivotPosition = pivotMotor.getPosition().getValueAsDouble();
        inputs.pivotVelocity = pivotMotor.getVelocity().getValueAsDouble();
        inputs.pivotVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotAmps = pivotMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setPivotPosition(double targetPosition) {
        pivotMotor.setControl(new MotionMagicVoltage(targetPosition));
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void resetPosition() {
        pivotMotor.setPosition(0);
    }
}
