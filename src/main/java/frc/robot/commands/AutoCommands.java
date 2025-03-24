package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.coral.CoralConstants;
import frc.robot.subsystems.algae.AlgaeConstants;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** A class containing utility command sequences for autonomous */
public class AutoCommands {
    public static void createIntakeSequence(AutoTrajectory base, RobotContainer robot) {
        createIntakeSequence(base, (Command) null, robot);
    }

    public static void createIntakeSequence(AutoTrajectory base, AutoTrajectory onComplete, RobotContainer robot) {
        createIntakeSequence(base, onComplete.cmd(), robot);
    }

    /**
     * Creates an intake sequence at the end of a trajectory,
     * running another trajectory after the sequence is complete
     * @param base - the trajectory that the intake commands should be attached to
     * @param onComplete - the trajectory that will be run after intaking
     * @param robot - the robot container instance
     */
    public static void createIntakeSequence(AutoTrajectory base, Command onComplete, RobotContainer robot) {
        base.done().onTrue(robot.algae.setPivotPosition(AlgaeConstants.intake)
            .andThen(robot.coral.intakeUntilFunnelEnter()));
        if (onComplete != null) {
            base.recentlyDone().and(robot.coral.funnelSensor).onTrue(
                onComplete.deadlineFor(robot.algae.setPivotPosition(AlgaeConstants.stow)
                    .andThen(robot.coral.completeIntaking())));
        } else {
            base.recentlyDone().and(robot.coral.funnelSensor).onTrue(
                robot.algae.setPivotPosition(AlgaeConstants.stow)
                    .andThen(robot.coral.completeIntaking()));
        }
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
        base.doneDelayed(0.44).onTrue(Commands.sequence(
            robot.coral.setVoltageCommand(CoralConstants.l4Speed).repeatedly().withTimeout(duration)
            .andThen(robot.coral.setVoltageCommand(0))
            .andThen(robot.elevator.stowCommand()
            .andThen(Commands.waitSeconds(0.5))
            .andThen(robot.coral.intakeUntilFunnelEnter()))
        ));
        base.doneDelayed(0.48 + duration).onTrue(onComplete);
    }
}
