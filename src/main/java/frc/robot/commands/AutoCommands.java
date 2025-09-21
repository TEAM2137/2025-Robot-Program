package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.autoalign.AutoAlign;
import frc.robot.autoalign.AutoAlign.Target;
import frc.robot.subsystems.coral.CoralConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.FieldPOIs;
import frc.robot.subsystems.algae.AlgaeConstants;
import choreo.auto.AutoTrajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** A class containing utility command sequences for autonomous */
public class AutoCommands {
    /**
     * Creates an intake sequence at the end of a trajectory using auto align to the station,
     * running another trajectory after the sequence is complete
     * @param base - the trajectory that the intake commands should be attached to
     * @param onComplete - the trajectory that will be run after intaking
     * @param robot - the robot container instance
     */
    public static void createIntakeSequenceAutoAlign(double beforeEnd, AutoTrajectory base, AutoTrajectory onComplete, RobotContainer robot) {
        base.atTimeBeforeEnd(beforeEnd).onTrue(new SequentialCommandGroup(
            robot.algae.setPivotPosition(AlgaeConstants.intake),
            robot.coral.intakeUntilFunnelEnter().andThen(Commands.waitSeconds(0.25)).deadlineFor(
                AutoAlign.driveToTargetCommand(robot).beforeStarting(() -> AutoAlign.setTargetPose(
                    (robot.drive.getPose().getY() < 8.19912 / 2.0 == !ChoreoAllianceFlipUtil.shouldFlip())
                        ? AutoAlign.flipIfRed(FieldPOIs.CORAL_STATION_BOTTOM)
                        : AutoAlign.flipIfRed(FieldPOIs.CORAL_STATION_TOP)
                ))
            ),
            (onComplete != null)
                ? robot.algae.setPivotPosition(AlgaeConstants.stow).andThen(onComplete.cmd().asProxy())
                : robot.algae.setPivotPosition(AlgaeConstants.stow).andThen(robot.coral.completeIntaking())
        ));
        if (onComplete != null) onComplete.active().onTrue(robot.coral.completeIntaking());
    }

    /**
     * Creates a scoring sequence at the end of a trajectory,
     * running another trajectory after the sequence is complete
     * @param duration - the time in seconds that the coral rollers should run for
     * @param base - the trajectory that the scoring commands should be attached to
     * @param onComplete - the trajectory that will be run after scoring
     * @param robot - the robot container instance
     */
    public static void createScoringSequence(double duration, AutoTrajectory base, AutoTrajectory onComplete, RobotContainer robot) {
        createScoringSequence(duration, base, onComplete.cmd(), robot);
    }

    /**
     * Creates a scoring sequence at the end of a trajectory,
     * running a command after the sequence is complete
     * @param duration - the time in seconds that the coral rollers should run for
     * @param base - the trajectory that the scoring commands should be attached to
     * @param onComplete - the command that will be run after scoring
     * @param robot - the robot container instance
     */
    public static void createScoringSequence(double duration, AutoTrajectory base, Command onComplete, RobotContainer robot) {
        base.doneDelayed(0.48).onTrue(Commands.sequence(
            robot.coral.setVelocityCommand(CoralConstants.l4RadPerSec).repeatedly().withTimeout(duration)
            .andThen(robot.coral.setVoltageCommand(0))
            .andThen(robot.elevator.setPositionCommand(ElevatorConstants.stow))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(robot.coral.intakeUntilFunnelEnter())
        ));
        base.doneDelayed(0.52 + duration).onTrue(onComplete);
    }

    public static Command scoreWithAutoAlign(double alignDelay, boolean flipAligns, double duration, Target targetType, AutoTrajectory base, Command onComplete, RobotContainer robot) {
        Target flippedTarget = targetType == Target.LEFT_BRANCH && flipAligns ? Target.RIGHT_BRANCH
            : (targetType == Target.RIGHT_BRANCH && flipAligns ? Target.LEFT_BRANCH : targetType);
        return new SequentialCommandGroup(
            new SequentialCommandGroup(
                Commands.waitSeconds(alignDelay + 0.4),
                robot.coral.setVelocityCommand(CoralConstants.l4RadPerSec).repeatedly().withTimeout(duration)
            ).deadlineFor(
                AutoAlign.driveToTargetWithElevatorCommand(robot).beforeStarting(() -> {
                    AutoAlign.setTargetPose(AutoAlign.flipIfRed(AutoAlign.fromPoseId(
                        AutoAlign.getNearestPose(base.getFinalPose().orElse(new Pose2d()),
                            new Translation2d(), AutoAlign.getPosesFor(flippedTarget), flippedTarget), flippedTarget)
                    ));
                })
            ),
            new SequentialCommandGroup(
                robot.coral.setVoltageCommand(0),
                robot.elevator.setPositionCommand(ElevatorConstants.stow),
                onComplete.asProxy()
            )
        );
    }
}
