package frc.robot.commands;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoCommands {
    public static Command intakeThenDo(Command driveAwayCommand, double timeoutSeconds, RobotContainer robot) {
        return Commands.sequence(
            // Intake the coral
            robot.coral.intakeUntilBrokenCommand(timeoutSeconds),

            // Start moving after the beam is broken, but complete the intake while driving away
            Commands.parallel(
                driveAwayCommand,
                robot.coral.intakeUntilNotBrokenCommand(timeoutSeconds / 2.0)
            )
        );
    }

    // TODO
    public static Command score() {
        return Commands.none();
    }
}
