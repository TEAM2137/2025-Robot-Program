package frc.robot.subsystems.coral;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class CoralIOTalonFX implements CoralIO {
    public TalonFX wheels;

    public CoralIOTalonFX() {
        wheels = new TalonFX(0);
    }

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        inputs.appliedVolts = wheels.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = wheels.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setRollerVoltage(double voltage) {
        wheels.setControl(new VoltageOut(voltage));
    }
}
