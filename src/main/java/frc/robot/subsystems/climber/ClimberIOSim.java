package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberIOSim implements ClimberIO {
    public final SingleJointedArmSim armSim;
    public final PIDController pid = new PIDController(60.0, 0, 18.0);
    public final ArmFeedforward ff = new ArmFeedforward(0.0, 0, 0.0);

    private double appliedVolts = 0.0;

    public ClimberIOSim() {
        armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            3.0, 1.0,
            0.3, 0,
            Math.PI / 2.0, false,
            0
        );
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        double feedForward = ff.calculate(armSim.getAngleRads(), armSim.getVelocityRadPerSec());
        appliedVolts = MathUtil.clamp(pid.calculate(armSim.getAngleRads()) + feedForward, -12.0, 12.0);
        armSim.setInputVoltage(appliedVolts);
        armSim.update(0.02);

        inputs.armAngle = Rotation2d.fromRadians(armSim.getAngleRads());
        inputs.velocityRadPerSec = armSim.getVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = armSim.getCurrentDrawAmps();
    }

    @Override
    public void setPosition(double targetPosition) {
        pid.setSetpoint(Math.toRadians(targetPosition));
    }
}
