package frc.robot.commands;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoCommands {
    public static Command autoIntakeCommand(Command driveAwayCommand, double intakeTimeout, RobotContainer robot) {
        return Commands.sequence(
            // Intake the coral
            robot.coral.intakeUntilBrokenCommand(intakeTimeout),

            // Start moving after the beam is broken, but complete the intake while driving away
            Commands.parallel(
                driveAwayCommand,
                robot.coral.intakeUntilNotBrokenCommand(intakeTimeout / 2.0)
            )
        );
    }
}
