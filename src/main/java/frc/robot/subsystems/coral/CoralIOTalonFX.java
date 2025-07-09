package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;

public class CoralIOTalonFX implements CoralIO {
    public TalonFX rollers = new TalonFX(CoralConstants.rollersID, "rio");
    public CANrange endEffectorSensor = new CANrange(CoralConstants.endEffectorSensorID, "rio");
    public CANrange funnelSensor = new CANrange(CoralConstants.funnelSensorID, "rio");
    public CANrange coralSensor = new CANrange(CoralConstants.coralSensorID, "rio");

    private VelocityVoltage velocityControl = new VelocityVoltage(0.0);
    private VoltageOut voltageControl = new VoltageOut(0.0);

    public CoralIOTalonFX() {
        // Create TalonFX config
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var slot0Configs = config.Slot0;
        slot0Configs.kP = CoralConstants.kP;
        slot0Configs.kS = CoralConstants.kS;
        slot0Configs.kV = CoralConstants.kV;

        // Set frequencies of used signals
        BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(100),
            rollers.getVelocity(),
            rollers.getMotorVoltage(),
            rollers.getSupplyCurrent()
        );

        rollers.getDeviceTemp().setUpdateFrequency(4);

        // Apply configurations
        rollers.getConfigurator().apply(config);
        rollers.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        inputs.velocityRadPerSec = rollers.getVelocity().getValue().in(RadiansPerSecond);
        inputs.targetVelocityRadPerSec = RotationsPerSecond.of(velocityControl.Velocity).in(RadiansPerSecond);

        inputs.appliedVolts = rollers.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = rollers.getSupplyCurrent().getValueAsDouble();

        inputs.motorTemperature = rollers.getDeviceTemp().getValueAsDouble();
        inputs.thermalShutdown = rollers.getFault_DeviceTemp().getValue();

        StatusSignal<Distance> eeDistance = endEffectorSensor.getDistance();
        inputs.endEffectorConnected = BaseStatusSignal.refreshAll(eeDistance).equals(StatusCode.OK);
        inputs.endEffectorDistanceCm = endEffectorSensor.getDistance().getValue().in(Centimeters);

        StatusSignal<Distance> fDistance = funnelSensor.getDistance();
        inputs.funnelConnected = BaseStatusSignal.refreshAll(fDistance).equals(StatusCode.OK);
        inputs.funnelDistanceCm = fDistance.getValue().in(Centimeters);

        StatusSignal<Distance> cDistance = coralSensor.getDistance();
        inputs.coralSensorConnected = BaseStatusSignal.refreshAll(cDistance).equals(StatusCode.OK);
        inputs.coralDistanceCm = cDistance.getValue().in(Centimeters);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollers.setControl(voltageControl.withOutput(Volts.of(voltage)));
    }

    @Override
    public void setRollerVelocity(double velocityRadPerSec) {
        rollers.setControl(velocityControl.withVelocity(RadiansPerSecond.of(velocityRadPerSec).in(RotationsPerSecond)));
    }
}
