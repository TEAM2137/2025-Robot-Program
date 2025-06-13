package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autoalign.AutoAlign;
import frc.robot.autoalign.AutoAlign.Target;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.algae.AlgaeConstants;
import frc.robot.subsystems.coral.CoralConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class Autonomous {
    public final AutoFactory factory;

    private final Map<String, Pose2d> startPoses = new HashMap<>();
    private final Trigger dsAttached = new Trigger(DriverStation::isDSAttached);

    private final LoggedDashboardChooser<Command> sysIdCommandChooser;
    private final LoggedDashboardChooser<AutoRoutine> autoChooser;

    private final RobotContainer robot;
    private final Drive drive;

    public Autonomous(RobotContainer robot) {
        this.robot = robot;
        this.drive = robot.drive;

        // Configure Choreo AutoFactory
        this.factory = new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::setPose, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive, // The drive subsystem
            (trajectory, starting) -> {
                // Log the supplied trajectory
                drive.fieldTrajectory.setTrajectory(new Trajectory(trajectory.samples().stream()
                    .map(sample -> new Trajectory.State(
                        sample.getTimestamp(),
                        new Translation2d(sample.vx, sample.vy).getNorm(),
                        new Translation2d(sample.ax, sample.ay).getNorm(),
                        AutoAlign.flipIfRed(sample.getPose()),
                        sample.omega
                    )).collect(Collectors.toList())
                ));
                Logger.recordOutput("Autonomous/AutoTrajectory", Arrays.stream(trajectory.getPoses())
                    .map(pose -> AutoAlign.flipIfRed(pose).getTranslation())
                    .collect(Collectors.toList())
                    .toArray(new Translation2d[0])
                );
            }
        );

        // Create the sysId command chooser
        this.sysIdCommandChooser = new LoggedDashboardChooser<>("SysID Command Chooser");
        this.sysIdCommandChooser.addDefaultOption("None", null);

        // Create the auto chooser
        this.autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
        this.autoChooser.addDefaultOption("None", null);
        this.registerAutos();

        // Assign auto commands to autonomous trigger
        RobotModeTriggers.autonomous().whileTrue(getSelectedAuto());
        RobotModeTriggers.autonomous().onFalse(robot.coral.setVoltageCommand(0).ignoringDisable(true));
    }

    /** @return A command to schedule the auto selected on the chooser */
    public Command getSelectedAuto() {
        return Commands.defer(() -> {
            if (autoChooser.get() != null) return autoChooser.get().cmd().asProxy();
            else if (sysIdCommandChooser.get() != null) return sysIdCommandChooser.get().asProxy();
            else return Commands.none();
        }, Set.of());
    }

    public Optional<Pose2d> getStartPose() {
        String selectedAuto = autoChooser.getSendableChooser().getSelected();
        return Optional.ofNullable(startPoses.get(selectedAuto));
    }

    /** Adds autos to the chooser */
    public void registerAutos() {
        // SysId routines
        sysIdCommandChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        sysIdCommandChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        sysIdCommandChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysIdCommandChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysIdCommandChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysIdCommandChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Testing Autos
        registerAuto("4 Coral Right", this::fourCoral);
        registerAuto("4 Coral Right (Reversed)", this::fourCoral);
        registerAuto("3 Coral Left", this::threeCoral);
        registerAuto("3 Coral Right", this::threeCoral);
        registerAuto("Center 3 Piece", this::centerThreePiece);
    }

    public void registerAuto(String name, Function<String, AutoRoutine> auto) {
        autoChooser.addOption(name, auto.apply(name));
    }

    public void registerAuto(String name, Supplier<AutoRoutine> auto) {
        autoChooser.addOption(name, auto.get());
    }

    public AutoRoutine driveStraight() {
        String pathName = "Drive Straight";
        AutoRoutine routine = factory.newRoutine(pathName);
        AutoTrajectory first = routine.trajectory(pathName, 0);

        routine.active().onTrue(Commands.sequence(
            first.resetOdometry(),
            first.cmd()
        ));

        return routine;
    }

    // Seconds that the coral rollers should run for when scoring
    private static final double scoreDuration = 0.4;

    private boolean targetAlgae = false;
    private boolean scoreNet = false;

    private Trigger targetAlgaeTrigger = new Trigger(() -> targetAlgae).and(RobotModeTriggers.autonomous());
    private Trigger scoreNetTrigger = new Trigger(() -> scoreNet).and(RobotModeTriggers.autonomous());

    @SuppressWarnings("deprecation")
    public AutoRoutine centerThreePiece(String dashboardName) {
        String pathName = "Center Coral Algae";
        AutoRoutine routine = factory.newRoutine(pathName);

        List<AutoTrajectory> splits = loadSplits(routine, pathName, 8);
        AutoTrajectory toReef1 = splits.get(0);
        AutoTrajectory backUp = splits.get(1);
        AutoTrajectory toNet1 = splits.get(3);
        AutoTrajectory toReef2 = splits.get(4);
        AutoTrajectory toNet2 = splits.get(6);
        AutoTrajectory offLine = splits.get(7);

        // Add start pose to map
        dsAttached.onTrue(Commands.runOnce(() -> toReef1.getInitialPose()
            .ifPresent(pose -> startPoses.put(dashboardName, pose))).ignoringDisable(true));

        targetAlgaeTrigger.onTrue(robot.createAlgaeAlign(true)); // TODO fix high/low
        robot.netTossWhen(scoreNetTrigger);

        routine.active().onTrue(robot.elevator.resetPositionCommand().andThen(robot.algae.setPivotPosition(AlgaeConstants.stow)));
        routine.active().onTrue(toReef1.resetOdometry().andThen(toReef1.cmd()));

        toReef1.atTimeBeforeEnd(0.7).onTrue(
            robot.elevator.setPositionCommand(ElevatorConstants.L4));

        AutoCommands.createScoringSequence(scoreDuration, toReef1, backUp, robot);

        backUp.done().onTrue(new SequentialCommandGroup(
            Commands.runOnce(() -> targetAlgae = true),
            Commands.waitSeconds(1.2),
            Commands.runOnce(() -> targetAlgae = false),
            toNet1.cmd().asProxy()
        ));

        toNet1.atTime(0.7).onTrue(robot.elevator.setPositionCommand(ElevatorConstants.stow)
            .andThen(Commands.waitSeconds(0.5))
            .andThen(robot.coral.setVoltageCommand(CoralConstants.algaeHoldVoltage)));

        toNet1.done().onTrue(Commands.runOnce(() -> scoreNet = true)
            .andThen(Commands.waitSeconds(1.0))
            .andThen(Commands.runOnce(() -> scoreNet = false))
            .andThen(toReef2.cmd().asProxy()));

        toReef2.atTimeBeforeEnd(0.4).onTrue(new SequentialCommandGroup(
            Commands.runOnce(() -> targetAlgae = true),
            Commands.waitSeconds(1.2),
            Commands.runOnce(() -> targetAlgae = false),
            toNet2.cmd().asProxy()
        ));

        toNet2.atTime(0.7).onTrue(robot.elevator.setPositionCommand(ElevatorConstants.stow)
            .andThen(Commands.waitSeconds(0.5))
            .andThen(robot.coral.setVoltageCommand(CoralConstants.algaeHoldVoltage)));

        toNet2.done().onTrue(Commands.runOnce(() -> scoreNet = true)
            .andThen(Commands.waitSeconds(1.0))
            .andThen(Commands.runOnce(() -> scoreNet = false))
            .andThen(offLine.cmd().asProxy()));

        return routine;
    }

    public AutoRoutine threeCoral(String dashboardName) {
        boolean flipAligns = dashboardName.contains("Left");
        String pathName = "3 Coral " + (flipAligns ? "Upper" : "Lower");
        AutoRoutine routine = factory.newRoutine(pathName);

        // Load the routine's trajectories
        List<AutoTrajectory> splits = loadSplits(routine, pathName, 7);
        AutoTrajectory toReef1 = splits.get(0);
        AutoTrajectory toStation2 = splits.get(1);
        AutoTrajectory toReef2 = splits.get(2);
        AutoTrajectory toStation3 = splits.get(3);
        AutoTrajectory toReef3 = splits.get(4);
        AutoTrajectory toStation4 = splits.get(5);

        // Add start pose to map
        dsAttached.onTrue(Commands.runOnce(() -> toReef1.getInitialPose()
            .ifPresent(pose -> startPoses.put(dashboardName, pose))).ignoringDisable(true));

        double alignDelay = 0.7;
        double intakeDelay = 0.5;

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(robot.elevator.resetPositionCommand()
            .andThen(robot.algae.setPivotPosition(AlgaeConstants.stow))
            .andThen(robot.elevator.schedulePositionCommand(ElevatorConstants.L4)));
        routine.active().onTrue(toReef1.resetOdometry().andThen(toReef1.cmd()));

        // Score coral 1 and drive to pickup coral
        toReef1.atTimeBeforeEnd(alignDelay).onTrue(AutoCommands.scoreWithAutoAlign(alignDelay,
            flipAligns, scoreDuration, Target.LEFT_BRANCH, toReef1, toStation2.cmd(), robot));

        // Intake coral 2 from coral station, then drive to reef
        AutoCommands.createIntakeSequenceAutoAlign(intakeDelay, toStation2, toReef2, robot);

        // Score coral 2 and drive to pickup coral
        toReef2.atTimeBeforeEnd(alignDelay).onTrue(AutoCommands.scoreWithAutoAlign(alignDelay,
            flipAligns, scoreDuration, Target.RIGHT_BRANCH, toReef2, toStation3.cmd(), robot));

        // Intake coral 3 from coral station, then drive to reef
        AutoCommands.createIntakeSequenceAutoAlign(intakeDelay, toStation3, toReef3, robot);

        // Score coral 3 and drive to pickup coral
        toReef3.atTimeBeforeEnd(alignDelay).onTrue(AutoCommands.scoreWithAutoAlign(alignDelay,
            flipAligns, scoreDuration, Target.LEFT_BRANCH, toReef3, toStation4.cmd(), robot));

        // Intake coral 4 from coral station
        AutoCommands.createIntakeSequenceAutoAlign(intakeDelay, toStation4, null, robot);

        return routine;
    }

    public AutoRoutine fourCoral(String dashboardName) {
        boolean flipAligns = dashboardName.contains("Left");
        String pathName = "4 Coral " + (flipAligns ? "Upper" : "Lower");
        AutoRoutine routine = factory.newRoutine(pathName);

        // Load the routine's trajectories
        List<AutoTrajectory> splits = loadSplits(routine, pathName, 7);
        AutoTrajectory toReef1 = splits.get(0);
        AutoTrajectory toStation2 = splits.get(1);
        AutoTrajectory toReef2 = splits.get(2);
        AutoTrajectory toStation3 = splits.get(3);
        AutoTrajectory toReef3 = splits.get(4);
        AutoTrajectory toStation4 = splits.get(5);
        AutoTrajectory toReef4 = splits.get(6);

        // Add start pose to map
        dsAttached.onTrue(Commands.runOnce(() -> toReef1.getInitialPose()
            .ifPresent(pose -> startPoses.put(dashboardName, pose))).ignoringDisable(true));

        double alignDelay = 0.9;
        double intakeDelay = 0.7;

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(robot.elevator.resetPositionCommand()
            .andThen(robot.algae.setPivotPosition(AlgaeConstants.stow))
            .andThen(robot.elevator.schedulePositionCommand(ElevatorConstants.L4)));
        routine.active().onTrue(toReef1.resetOdometry().andThen(toReef1.cmd()));

        // Score coral 1 and drive to pickup coral
        toReef1.atTimeBeforeEnd(alignDelay).onTrue(AutoCommands.scoreWithAutoAlign(alignDelay,
            flipAligns, scoreDuration, Target.LEFT_BRANCH, toReef1, toStation2.cmd(), robot));

        // Intake coral 2 from coral station, then drive to reef
        AutoCommands.createIntakeSequenceAutoAlign(intakeDelay, toStation2, toReef2, robot);

        // Score coral 2 and drive to pickup coral
        toReef2.atTimeBeforeEnd(alignDelay).onTrue(AutoCommands.scoreWithAutoAlign(alignDelay,
            flipAligns, scoreDuration, Target.RIGHT_BRANCH, toReef2, toStation3.cmd(), robot));

        // Intake coral 3 from coral station, then drive to reef
        AutoCommands.createIntakeSequenceAutoAlign(intakeDelay, toStation3, toReef3, robot);

        // Score coral 3 and drive to pickup coral
        toReef3.atTimeBeforeEnd(alignDelay).onTrue(AutoCommands.scoreWithAutoAlign(alignDelay,
            flipAligns, scoreDuration, Target.LEFT_BRANCH, toReef3, toStation4.cmd(), robot));

        // Intake coral 4 from coral station, then drive to reef
        AutoCommands.createIntakeSequenceAutoAlign(intakeDelay, toStation4, toReef4, robot);

        // Score coral 4
        toReef4.atTimeBeforeEnd(alignDelay).onTrue(AutoCommands.scoreWithAutoAlign(alignDelay,
            flipAligns, scoreDuration, Target.RIGHT_BRANCH, toReef4, Commands.none(), robot));

        return routine;
    }

    private ArrayList<AutoTrajectory> loadSplits(AutoRoutine routine, String path, int numSplits) {
        ArrayList<AutoTrajectory> splits = new ArrayList<>();
        for (int i = 0; i < numSplits; i++) {
            splits.add(routine.trajectory(path, i));
        }
        return splits;
    }

    public static String getSetupScore(Pose2d pose, Pose2d targetPose) {
        double positionError = pose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError = Math.abs(pose.getRotation().minus(targetPose.getRotation()).getDegrees());

        Logger.recordOutput("Autonomous/Setup/PosError", positionError);
        Logger.recordOutput("Autonomous/Setup/RotError", rotationError);

        double rawScore = 100 * Math.exp(-(1.0 * positionError + 0.008 * rotationError));
        int scoreRounded = (int) Math.round(rawScore);
        double scoreNearestHundreth = ((int) (rawScore * 10.0)) / 10.0;

        String letterGrade = "F";
        if (scoreRounded >= 97) letterGrade = "A+";
        else if (scoreRounded >= 93) letterGrade = "A";
        else if (scoreRounded >= 90) letterGrade = "A-";
        else if (scoreRounded >= 87) letterGrade = "B+";
        else if (scoreRounded >= 83) letterGrade = "B";
        else if (scoreRounded >= 80) letterGrade = "B-";
        else if (scoreRounded >= 77) letterGrade = "C+";
        else if (scoreRounded >= 73) letterGrade = "C";
        else if (scoreRounded >= 70) letterGrade = "C-";
        else if (scoreRounded >= 67) letterGrade = "D+";
        else if (scoreRounded >= 63) letterGrade = "D";
        else if (scoreRounded >= 60) letterGrade = "D-";

        return letterGrade + ": " + scoreNearestHundreth + "%";
    }
}
