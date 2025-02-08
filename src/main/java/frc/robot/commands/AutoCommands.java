package frc.robot.commands;

import frc.robot.RobotContainer;
import choreo.auto.AutoTrajectory;

public class AutoCommands {
    /**
     *
     * @param toStation - the trajectory that the intake commands should be attached to
     * @param toReef - the trajectory that will be run after intaking
     * @param robot - the robot container instance
     */
    public static void createIntakeSequence(AutoTrajectory toStation, AutoTrajectory toReef, RobotContainer robot) {
        toStation.done().onTrue(robot.coral.intakeUntilBrokenCommand());
        robot.coral.beamBroken.and(toStation.recentlyDone()).onTrue(
            robot.coral.intakeWhileBrokenCommand().alongWith(toReef.cmd()));
    }
}
