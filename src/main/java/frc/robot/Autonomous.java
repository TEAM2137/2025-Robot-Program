package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoAlignUtil;

public class Autonomous {
    public final AutoFactory factory;

    private final LoggedDashboardChooser<Command> sysIdCommandChooser;
    private final LoggedDashboardChooser<AutoRoutine> autoChooser;

    private final RobotContainer robot;
    private final Drive drive;

    private static StructArrayPublisher<Translation2d> autoTrajectoryPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Autonomous/AutoTrajectory", Translation2d.struct).publish();

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
                autoTrajectoryPublisher.accept(Arrays.stream(trajectory.getPoses())
                    .map(pose -> AutoAlignUtil.flipIfRed(pose).getTranslation())
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
    }

    /** @return A command to schedule the auto selected on the chooser */
    public Command getSelectedAuto() {
        return Commands.defer(() -> {
            if (autoChooser.get() != null) return autoChooser.get().cmd().asProxy();
            else if (sysIdCommandChooser.get() != null) return sysIdCommandChooser.get().asProxy();
            else return Commands.none();
        }, Set.of());
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
        autoChooser.addOption("4 Coral Upper", fourCoralUpper());
        autoChooser.addOption("Drive Forward", driveStraight());
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

    public AutoRoutine fourCoralUpper() {
        String pathName = "4 Coral Upper";
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

        // Seconds before the end of the path that the elevator should raise
        // double elevatorDelay = 0.8;
        // Seconds that the coral rollers should run for when scoring
        double scoreDuration = 0.5;

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(toReef1.resetOdometry()
            // .andThen(robot.elevator.resetPositionCommand())
            .andThen(toReef1.cmd()));

        // Raise elevator on approach
        // toReef1.atTimeBeforeEnd(elevatorDelay).onTrue(
        //     robot.elevator.setPositionCommand(ElevatorConstants.L4));

        // Score coral 1, stow elevator, and drive to pickup coral
        AutoCommands.createScoringSequence(scoreDuration, toReef1, toStation2, robot);

        // Intake coral 2 from coral station, then drive to reef
        AutoCommands.createIntakeSequence(toStation2, toReef2, robot);

        // Raise elevator on approach
        // toReef2.atTimeBeforeEnd(elevatorDelay).onTrue(
        //     robot.elevator.setPositionCommand(ElevatorConstants.L4));

        // Score coral 2, stow elevator, and drive to pickup coral
        AutoCommands.createScoringSequence(scoreDuration, toReef2, toStation3, robot);

        // Intake coral 3 from coral station, then drive to reef
        AutoCommands.createIntakeSequence(toStation3, toReef3, robot);

        // Raise elevator on approach
        // toReef3.atTimeBeforeEnd(elevatorDelay).onTrue(
        //     robot.elevator.setPositionCommand(ElevatorConstants.L4));

        // Score coral 3, stow elevator, and drive to pickup coral
        AutoCommands.createScoringSequence(scoreDuration, toReef3, toStation4, robot);

        // Intake coral 4 from coral station, then drive to reef
        AutoCommands.createIntakeSequence(toStation4, toReef4, robot);

        // Raise elevator on approach
        // toReef4.atTimeBeforeEnd(elevatorDelay).onTrue(
        //     robot.elevator.setPositionCommand(ElevatorConstants.L4));

        // Score coral 4, stow elevator, and drive to pickup coral
        AutoCommands.createScoringSequence(scoreDuration, toReef1, robot);

        return routine;
    }

    private ArrayList<AutoTrajectory> loadSplits(AutoRoutine routine, String path, int numSplits) {
        ArrayList<AutoTrajectory> splits = new ArrayList<>();
        for (int i = 0; i < numSplits; i++) {
            splits.add(routine.trajectory(path, i));
        }
        return splits;
    }
}
