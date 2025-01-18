package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim sim;
    private PIDController pid = new PIDController(30.0, 0.0, 3.0);
    private ElevatorFeedforward ff = new ElevatorFeedforward(0.0, 6.0, 0.0);

    private boolean pidControlEnabled = false;
    private double appliedVolts = 0.0;

    public ElevatorIOSim() {
        this.sim = new ElevatorSim(
            DCMotor.getKrakenX60(2),
            3.0, 7.257, 0.03175,
            0.26035, 1.96215,
            true, 0.26035
        );
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (pidControlEnabled) {
            double feedForward = ff.calculate(sim.getPositionMeters());
            appliedVolts = MathUtil.clamp(pid.calculate(sim.getPositionMeters()) + feedForward, -12.0, 12.0);
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
