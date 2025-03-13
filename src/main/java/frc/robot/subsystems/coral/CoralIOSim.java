package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CoralIOSim implements CoralIO {
    private DCMotorSim sim;

    public CoralIOSim(){
        this.sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),1.0,3.0),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        sim.update(0.02);

        inputs.appliedVolts = sim.getInputVoltage();
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.endEffectorDistanceCm = 0.0;
        inputs.endEffectorDistanceCm = 0.0;
        inputs.velocityRadPerSec = sim.getAngularVelocity().in(RadiansPerSecond);
        inputs.velocityRadPerSec = sim.getAngularVelocity().in(RadiansPerSecond);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        sim.setInputVoltage(voltage);
    }

    @Override
    public void setRollerVelocity(double velocityRadPerSec) {
        sim.setAngularVelocity(velocityRadPerSec);
    }
}
