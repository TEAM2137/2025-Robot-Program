package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX leadMotor = new TalonFX(0);
    private TalonFX followMotor = new TalonFX(1);

    public ElevatorIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.Slot0.kP = ElevatorConstants.kPDefault;
        config.Slot0.kD = ElevatorConstants.kDDefault;
        config.Feedback.RotorToSensorRatio = ElevatorConstants.gearing;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        leadMotor.getConfigurator().apply(config);
        leadMotor.optimizeBusUtilization();

        followMotor.getConfigurator().apply(config);        
        followMotor.optimizeBusUtilization();

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
}
