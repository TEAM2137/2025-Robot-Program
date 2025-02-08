package frc.robot.subsystems.coral;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CoralIOSim implements CoralIO {
    private DCMotorSim sim;
    private boolean isBeamBroken = false;

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

        // The only solution I could come up with for making sim autos always work
        isBeamBroken = !isBeamBroken;

        inputs.appliedVolts = sim.getInputVoltage();
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.isBeamBroken = isBeamBroken;
    }

    @Override
    public boolean isBeamBroken() {
        return isBeamBroken;
    }

    @Override
    public void setRollerVoltage(double voltage) {
        sim.setInputVoltage(voltage);
    }
}
