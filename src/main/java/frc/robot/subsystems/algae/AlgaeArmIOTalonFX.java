package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class AlgaeArmIOTalonFX implements AlgaeArmIO {
    public TalonFX pivotMotor = new TalonFX(AlgaeConstants.deviceID, "rio");
    public CANcoder encoder = new CANcoder(AlgaeConstants.encoderID, "rio");
    public CANrange algaeSensor = new CANrange(AlgaeConstants.algaeSensorID, "rio");

    private double targetPosition;

    public AlgaeArmIOTalonFX() {
        var encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = AlgaeConstants.encoderOffset;
        encoder.getConfigurator().apply(encoderConfig);

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
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackConfigs.FeedbackRemoteSensorID = AlgaeConstants.encoderID;
        feedbackConfigs.RotorToSensorRatio = AlgaeConstants.gearing;

        // Current limit settings
        var currentLimitsConfigs = config.CurrentLimits;
        currentLimitsConfigs.StatorCurrentLimit = 80;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        // Motor output settings
        var motorOutputConfig = config.MotorOutput;
        motorOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
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

        algaeSensor.getConfigurator().apply(new ProximityParamsConfigs()
            .withProximityThreshold(AlgaeConstants.algaeSensorRange / 100.0));
    }

    @Override
    public double getTargetPosition() {
        return this.targetPosition;
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        inputs.pivotPosition = pivotMotor.getPosition().getValueAsDouble();
        inputs.pivotVelocity = pivotMotor.getVelocity().getValueAsDouble();
        inputs.pivotVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotAmps = pivotMotor.getSupplyCurrent().getValueAsDouble();

        StatusSignal<Boolean> algaeIsDetected = algaeSensor.getIsDetected();
        inputs.algaeSensorConnected = BaseStatusSignal.refreshAll(algaeIsDetected).equals(StatusCode.OK);
        inputs.algaeIsDetected = algaeSensor.getIsDetected().getValue();
    }

    @Override
    public void setPivotPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        pivotMotor.setControl(new MotionMagicVoltage(targetPosition));
    }

    @Override
    public void setPivotVoltage(double voltage) {
        this.targetPosition = AlgaeConstants.stow;
        pivotMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void resetPosition() {
        this.targetPosition = AlgaeConstants.stow;
        pivotMotor.setPosition(0);
        setPivotPosition(0.0);
    }

    @Override
    public double getPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }
}
