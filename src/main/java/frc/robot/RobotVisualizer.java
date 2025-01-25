package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class RobotVisualizer extends SubsystemBase {

    private DoubleSupplier elevatorExtension;
    private Supplier<Rotation2d> cageRotation;

    public RobotVisualizer(DoubleSupplier elevatorExtension, Supplier<Rotation2d> cageRotation) {
        this.elevatorExtension = elevatorExtension;
        this.cageRotation = cageRotation;
    }

    @Override
    public void periodic() {
        // AScope 3d reprsentation

        var poses = new Pose3d[4];

        poses[0] = new Pose3d(0, 0, Math.max(elevatorExtension.getAsDouble()-(ElevatorConstants.stage2Range + ElevatorConstants.stage3Range), 0), new Rotation3d());
        poses[1] = new Pose3d(0, 0, Math.max(elevatorExtension.getAsDouble()-ElevatorConstants.stage3Range, 0), new Rotation3d());
        poses[2] = new Pose3d(0, 0, elevatorExtension.getAsDouble(), new Rotation3d());
        // poses[0] = new Pose3d();
        // poses[1] = new Pose3d();
        // poses[2] = new Pose3d();
        poses[3] = new Pose3d(0, -0.24093932, 0.22225, new Rotation3d(cageRotation.get().getRadians(), 0, 0));
        // poses[3] = new Pose3d();

        Logger.recordOutput("Visualization/AScope3d", poses);
    }
}
