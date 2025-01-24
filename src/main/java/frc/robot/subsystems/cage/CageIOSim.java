package frc.robot.subsystems.cage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class CageIOSim implements CageIO {
    public final SingleJointedArmSim cagearmSim;
    public final PIDController pid = new PIDController(60.0, 0, 18.0);
    public final ArmFeedforward ff = new ArmFeedforward(0.0, 0, 0.0);
    public final DCMotorSim rollersSim;

    private double appliedVolts = 0.0;

    public CageIOSim() {
        cagearmSim = new SingleJointedArmSim(
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
    public void updateInputs(CageIOInputs inputs) {
        double feedForward = ff.calculate(cagearmSim.getAngleRads(), cagearmSim.getVelocityRadPerSec());
        appliedVolts = MathUtil.clamp(pid.calculate(cagearmSim.getAngleRads()) + feedForward, -12.0, 12.0);
        cagearmSim.setInputVoltage(appliedVolts);
        cagearmSim.update(0.02);

        rollersSim.update(0.02);

        inputs.cagearmPosition = cagearmSim.getAngleRads();
        inputs.cagearmVelocity = cagearmSim.getVelocityRadPerSec();
        inputs.cagearmVoltage = appliedVolts;
        inputs.cagearmAmps = cagearmSim.getCurrentDrawAmps();
    }

    @Override
    public void setPosition(double targetPosition) {
        pid.setSetpoint(Math.toRadians(targetPosition));
    }
}
