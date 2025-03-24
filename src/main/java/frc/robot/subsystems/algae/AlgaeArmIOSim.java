package frc.robot.subsystems.algae;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AlgaeArmIOSim implements AlgaeArmIO {
    public final SingleJointedArmSim pivotSim;
    public final PIDController pid = new PIDController(60.0, 0, 18.0);
    public final ArmFeedforward ff = new ArmFeedforward(0.0, 0, 0.0);

    private double appliedVolts = 0.0;
    private boolean usePid;

    public AlgaeArmIOSim() {
        pivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            3.0, 1.0,
            0.3, -100,
            Math.PI / 2.0, false,
            0
        );
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        if (usePid) {
            double feedForward = ff.calculate(pivotSim.getAngleRads(), pivotSim.getVelocityRadPerSec());
            appliedVolts = MathUtil.clamp(pid.calculate(pivotSim.getAngleRads()) + feedForward, -12.0, 12.0);
            pivotSim.setInputVoltage(appliedVolts);
        }
        pivotSim.update(0.02);

        inputs.pivotPosition = pivotSim.getAngleRads();
        inputs.pivotVelocity = pivotSim.getVelocityRadPerSec();
        inputs.pivotVoltage = appliedVolts;
        inputs.pivotAmps = pivotSim.getCurrentDrawAmps();
    }

    @Override
    public double getPivotPosition() {
        return pivotSim.getAngleRads();
    }

    @Override
    public void setPivotVoltage(double voltage) {
        usePid = false;
        pivotSim.setInputVoltage(voltage);
    }

    @Override
    public void setPivotPosition(double targetPosition) {
        usePid = true;
        pid.setSetpoint(Math.toRadians(targetPosition));
    }
}
