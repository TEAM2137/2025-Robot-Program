package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim sim;
    private PIDController pid = new PIDController(1.0, 0.0, 0.1);

    private boolean pidControlEnabled = false;
    private double appliedVolts = 0.0;

    public ElevatorIOSim() {
        this.sim = new ElevatorSim(
            DCMotor.getKrakenX60(2),
            0.0, 7.0, 0.0,
            0.0, 1.0,
            true, 0.0
        );
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (pidControlEnabled) {
            appliedVolts = MathUtil.clamp(pid.calculate(sim.getPositionMeters()), -12.0, 12.0);
        }
        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);

        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMetersPerSecond = sim.getVelocityMetersPerSecond();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setTargetPosition(double targetPosition) {
        this.pid.setSetpoint(targetPosition);
        pidControlEnabled = true;
    }

    @Override
    public void setVolts(double appliedVolts) {
        this.appliedVolts = appliedVolts;
        pidControlEnabled = false;
    }
}
