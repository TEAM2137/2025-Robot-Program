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
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoAlignUtil;

public class Autonomous {
    public final AutoFactory factory;

    private final LoggedDashboardChooser<Command> sysIdCommandChooser;
    private final LoggedDashboardChooser<AutoRoutine> autoChooser;

    @SuppressWarnings("unused")
    private final RobotContainer robot;
    private final Drive drive;

    private static StructArrayPublisher<Translation2d> autoTrajectoryPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("AutoTrajectory", Translation2d.struct).publish();

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
        AutoTrajectory trajectory = routine.trajectory(pathName);

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(Commands.sequence(
            trajectory.resetOdometry(),
            trajectory.cmd()
        ));

        return routine;
    }

    public AutoRoutine fourCoralUpper() {
        String pathName = "4 Coral Upper";
        AutoRoutine routine = factory.newRoutine(pathName);

        // Load the routine's trajectories
        List<AutoTrajectory> splits = loadSplits(routine, pathName, 7);

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(Commands.sequence(
            splits.get(0).resetOdometry(),
            splits.get(0).cmd(),
            splits.get(1).cmd(),
            splits.get(2).cmd(),
            splits.get(3).cmd(),
            splits.get(4).cmd(),
            splits.get(5).cmd(),
            splits.get(6).cmd()
        ));

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
