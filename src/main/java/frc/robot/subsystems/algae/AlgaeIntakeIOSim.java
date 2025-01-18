package frc.robot.subsystems.algae;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AlgaeIntakeIOSim implements AlgaeIntakeIO {
    public final SingleJointedArmSim pivotSim;
    public final PIDController pid = new PIDController(60.0, 0, 18.0);
    public final ArmFeedforward ff = new ArmFeedforward(0.0, 0, 0.0);
    public final DCMotorSim rollersSim;

    private double appliedVolts = 0.0;

    public AlgaeIntakeIOSim() {
        pivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            3.0, 1.0,
            0.3, 0,
            Math.PI / 2.0, false,
            0
        );
        rollersSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 1.0, 3.0),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        double feedForward = ff.calculate(pivotSim.getAngleRads(), pivotSim.getVelocityRadPerSec());
        appliedVolts = MathUtil.clamp(pid.calculate(pivotSim.getAngleRads()) + feedForward, -12.0, 12.0);
        pivotSim.setInputVoltage(appliedVolts);
        pivotSim.update(0.02);

        rollersSim.update(0.02);

        inputs.pivotPosition = pivotSim.getAngleRads();
        inputs.pivotVelocity = pivotSim.getVelocityRadPerSec();
        inputs.pivotVoltage = appliedVolts;
        inputs.pitvotAmps = pivotSim.getCurrentDrawAmps();
        inputs.rollerVoltage = rollersSim.getInputVoltage();
        inputs.rollerAmps = rollersSim.getCurrentDrawAmps();
    }

    @Override
    public void setPivotPosition(double targetPosition) {
        pid.setSetpoint(Math.toRadians(targetPosition));
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollersSim.setInputVoltage(voltage);
    }
}
