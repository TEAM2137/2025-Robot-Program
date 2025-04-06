package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim sim;
    private PIDController pid = new PIDController(27.0, 0.0, 2.5);
    private ElevatorFeedforward ff = new ElevatorFeedforward(0.0, 0.5, 0.0);

    private double scheduledPosition = 0.0;
    private boolean pidControlEnabled = false;
    private double appliedVolts = 0.0;

    public ElevatorIOSim() {
        this.sim = new ElevatorSim(
            DCMotor.getKrakenX60(2),
            3.0, 7.257, 0.03175,
            0.0, 1.82,
            true, 0.0
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

        inputs.scheduledTargetPosition = scheduledPosition;
        inputs.elevatorPositionMeters = sim.getPositionMeters();
        inputs.targetPositionMeters = pid.getSetpoint();
        inputs.velocityMetersPerSecond = sim.getVelocityMetersPerSecond();
        inputs.leaderOutputVolts = appliedVolts;
        inputs.leaderCurrentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void applyScheduledPosition() {
        this.setTargetPosition(scheduledPosition);
    }

    @Override
    public void schedulePosition(double targetPosition) {
        this.scheduledPosition = targetPosition;
    }

    @Override
    public double getScheduledPosition() {
        return this.scheduledPosition;
    }

    @Override
    public void resetPosition() {
        // sim.setState(0.26035, 0);
        setTargetPosition(0.0);
        sim.setState(0, 0);
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

    @Override
    public void setPIDConstants(double kP, double kD) {
        pid.setP(kP);
        pid.setD(kD);
    }
}
