package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX leadMotor = new TalonFX(ElevatorConstants.leaderID, "rio");
    private TalonFX followMotor = new TalonFX(ElevatorConstants.followerID, "rio");

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
        motionMagicConfigs.MotionMagicJerk = ElevatorConstants.targetJerk;

        // Other settings
        config.Feedback.RotorToSensorRatio = ElevatorConstants.gearing;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set frequencies of used signals
        BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(100),
            leadMotor.getPosition(),
            leadMotor.getVelocity(),
            leadMotor.getMotorVoltage(),
            leadMotor.getSupplyCurrent(),
            followMotor.getMotorVoltage(),
            followMotor.getSupplyCurrent()
        );

        // Apply configurations
        leadMotor.getConfigurator().apply(config);
        followMotor.getConfigurator().apply(config);

        // Clear unused signals
        leadMotor.optimizeBusUtilization();
        followMotor.optimizeBusUtilization();

        // Set followMotor to follow leadMotor
        followMotor.setControl(new Follower(leadMotor.getDeviceID(), true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.motorPositionRotations = leadMotor.getPosition().getValueAsDouble();
        inputs.elevatorPositionMeters = inputs.motorPositionRotations * ElevatorConstants.motorToElevatorPosition;
        inputs.velocityMetersPerSecond = leadMotor.getVelocity().getValueAsDouble();

        inputs.leaderAppliedVolts = leadMotor.getMotorVoltage().getValueAsDouble();
        inputs.leaderCurrentAmps = leadMotor.getSupplyCurrent().getValueAsDouble();

        inputs.followerAppliedVolts = followMotor.getMotorVoltage().getValueAsDouble();
        inputs.followerCurrentAmps = followMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setTargetPosition(double targetPosition) {
        leadMotor.setControl(new PositionVoltage(targetPosition));
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
}
