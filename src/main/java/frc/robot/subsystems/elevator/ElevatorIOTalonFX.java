package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX leadMotor = new TalonFX(ElevatorConstants.leaderID, "rio");
    private TalonFX followMotor = new TalonFX(ElevatorConstants.followerID, "rio");

    private double targetPosition = 0.0;
    private double scheduledPosition = 0.0;

    public ElevatorIOTalonFX() {
        // Create TalonFX config
        var config = new TalonFXConfiguration();

        // PID and FF settings
        var slot0Configs = config.Slot0;
        slot0Configs.kS = ElevatorConstants.kS;
        slot0Configs.kG = ElevatorConstants.kG;
        slot0Configs.kP = ElevatorConstants.kP;
        slot0Configs.kD = ElevatorConstants.kD;

        // Motion Magic settings
        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.targetCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.targetAcceleration;

        // Motor feedback settings
        var feedbackConfigs = config.Feedback;
        feedbackConfigs.RotorToSensorRatio = ElevatorConstants.gearing;

        // Current limit settings
        var currentLimitsConfigs = config.CurrentLimits;
        currentLimitsConfigs.StatorCurrentLimit = 80;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        // Motor output settings
        var motorOutputConfig = config.MotorOutput;
        motorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
        motorOutputConfig.NeutralMode = NeutralModeValue.Brake;

        // Apply configurations
        leadMotor.getConfigurator().apply(config);
        followMotor.getConfigurator().apply(config);

        // Set frequencies of used signals
        BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(100),
            leadMotor.getPosition(),
            leadMotor.getVelocity(),
            leadMotor.getMotorVoltage(),
            leadMotor.getSupplyCurrent(),
            followMotor.getSupplyCurrent(),
            followMotor.getMotorVoltage()
        );

        // Clear unused signals
        leadMotor.optimizeBusUtilization();
        followMotor.optimizeBusUtilization();

        // Set followMotor to follow leadMotor
        followMotor.setControl(new Follower(leadMotor.getDeviceID(), true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.motorPositionRotations = leadMotor.getPosition().getValueAsDouble();
        inputs.elevatorPositionMeters = inputs.motorPositionRotations * ElevatorConstants.spoolRadius * Math.PI;
        inputs.velocityMetersPerSecond = leadMotor.getVelocity().getValueAsDouble();

        inputs.scheduledTargetPosition = this.scheduledPosition;
        inputs.targetPositionMeters = targetPosition;
        inputs.targetPositionRotations = targetPosition / ElevatorConstants.spoolRadius / Math.PI;

        inputs.leaderOutputVolts = leadMotor.getMotorVoltage().getValueAsDouble();
        inputs.leaderCurrentAmps = leadMotor.getSupplyCurrent().getValueAsDouble();

        inputs.followerOutputVolts = followMotor.getMotorVoltage().getValueAsDouble();
        inputs.followerCurrentAmps = followMotor.getSupplyCurrent().getValueAsDouble();

        if (targetPosition == ElevatorConstants.stow && inputs.elevatorPositionMeters <= 0.01) setVolts(0);
    }

    @Override
    public void resetPosition() {
        leadMotor.setPosition(0.0);
        setTargetPosition(0.0);
    }

    @Override
    public void schedulePosition(double targetPosition) {
        this.scheduledPosition = targetPosition;
        if ((targetPosition == ElevatorConstants.L1 || this.targetPosition > 0.05)
            && DriverStation.isEnabled()) setTargetPosition(targetPosition);
    }

    @Override
    public void applyScheduledPosition() {
        setTargetPosition(this.scheduledPosition);
    }

    @Override
    public double getScheduledPosition() {
        return this.scheduledPosition;
    }

    @Override
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        leadMotor.setControl(new MotionMagicVoltage(targetPosition / ElevatorConstants.spoolRadius / Math.PI));
    }

    @Override
    public void setVolts(double appliedVolts) {
        leadMotor.setControl(new VoltageOut(appliedVolts));
    }

    @Override
    public void setPIDConstants(double kP, double kD) {
        leadMotor.getConfigurator().apply(new Slot0Configs().withKP(kP).withKD(kD));
    }

    @Override
    public void setFFConstants(double kS, double kG) {
        leadMotor.getConfigurator().apply(new Slot0Configs().withKS(kS).withKG(kG));
    }

    @Override
    public void setMMConstants(double v, double a, double j) {
        leadMotor.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(v)
            .withMotionMagicAcceleration(a)
            .withMotionMagicJerk(j));
    }

    @Override
    public boolean isAtTarget() {
        // TODO: tune the threshold
        return leadMotor.getClosedLoopError().getValueAsDouble() < 0.1f;
    }
}
