package frc.robot.commands;

import frc.robot.RobotContainer;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** A class containing utility command sequences for autonomous */
public class AutoCommands {
    /**
     * Creates an intake sequence at the end of a trajectory,
     * running another trajectory after the sequence is complete
     * @param base - the trajectory that the intake commands should be attached to
     * @param onComplete - the trajectory that will be run after intaking
     * @param robot - the robot container instance
     */
    public static void createIntakeSequence(AutoTrajectory base, AutoTrajectory onComplete, RobotContainer robot) {
        base.done().onTrue(robot.coral.intakeUntilBrokenCommand());
        base.recentlyDone().and(robot.coral.beamBroken).onTrue(
            robot.coral.intakeWhileBrokenCommand().alongWith(onComplete.cmd()));
        // base.doneDelayed(1.0).onTrue(onComplete.cmd());
    }

    /**
     * Creates a scoring sequence at the end of a trajectory,
     * running another trajectory after the sequence is complete
     * @param duration - the time in seconds that the coral rollers should run for
     * @param toReef - the trajectory that the scoring commands should be attached to
     * @param onComplete - the trajectory that will be run after scoring
     * @param robot - the robot container instance
     */
    public static void createScoringSequence(double duration, AutoTrajectory base, AutoTrajectory onComplete, RobotContainer robot) {
        createScoringSequence(duration, base, onComplete.cmd(), robot);
    }

    /**
     * Creates a scoring sequence at the end of a trajectory.
     * No commands will run when the sequence is complete
     * @param duration - the time in seconds that the coral rollers should run for
     * @param toReef - the trajectory that the scoring commands should be attached to
     * @param robot - the robot container instance
     */
    public static void createScoringSequence(double duration, AutoTrajectory base, RobotContainer robot) {
        createScoringSequence(duration, base, Commands.none(), robot);
    }

    /**
     * Creates a scoring sequence at the end of a trajectory,
     * running a command after the sequence is complete
     * @param duration - the time in seconds that the coral rollers should run for
     * @param toReef - the trajectory that the scoring commands should be attached to
     * @param onComplete - the command that will be run after scoring
     * @param robot - the robot container instance
     */
    public static void createScoringSequence(double duration, AutoTrajectory base, Command onComplete, RobotContainer robot) {
        base.doneDelayed(0.8).onTrue(Commands.sequence(
            robot.coral.setVoltageCommand(4).repeatedly().withTimeout(duration)
            .andThen(robot.coral.setVoltageCommand(0))
            .andThen(robot.elevator.stowCommand())
        ));

        base.doneDelayed(0.8 + duration + (duration / 2.0)).onTrue(onComplete);
    }
}
