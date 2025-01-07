package frc.robot.commands;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AutoCommands {
    public Command pickupAndScoreAuto(AutoFactory factory, RobotContainer subsystems) {
        return Commands.sequence(
            factory.resetOdometry("example-trajectory"),
            factory.trajectoryCmd("example-trajectory")
        );
    }
}
