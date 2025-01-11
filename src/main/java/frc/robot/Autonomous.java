package frc.robot;


import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GameEvents;

public class Autonomous {
    public final AutoFactory factory;

    private final AutoChooser chooser;
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
            drive // The drive subsystem
        );

        // Create the auto chooser
        this.chooser = new AutoChooser();
        this.registerAutos();

        // Put the auto chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", chooser);
        chooser.select("Example Auto (5 cycles)");

        // Assign auto commands to autonomous trigger
        GameEvents.autonomous().whileTrue(getSelectedAuto());
    }

    /** @return A command to schedule the auto selected on the chooser */
    public Command getSelectedAuto() {
        return chooser.selectedCommandScheduler();
    }

    /** Adds autos to the chooser */
    public void registerAutos() {
        // SysId routines
        chooser.addCmd("Drive Wheel Radius Characterization", () -> DriveCommands.wheelRadiusCharacterization(drive));
        chooser.addCmd("Drive Simple FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
        chooser.addCmd("Drive SysId (Quasistatic Forward)", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        chooser.addCmd("Drive SysId (Quasistatic Reverse)", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        chooser.addCmd("Drive SysId (Dynamic Forward)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        chooser.addCmd("Drive SysId (Dynamic Reverse)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Testing Autos
        chooser.addRoutine("Example Auto (5 cycles)", () -> exampleAuto(5));
    }

    public AutoRoutine exampleAuto(int cycles) {
        AutoRoutine routine = factory.newRoutine("example-auto");

        // Load the routine's trajectories
        AutoTrajectory scorePreloaded = routine.trajectory("score-preloaded");
        AutoTrajectory preloadToCoral = routine.trajectory("preload-to-coral-station");
        AutoTrajectory coralToReef = routine.trajectory("coral-station-to-reef");

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(
            cycleAfter(cycles - 1, Commands.sequence(
                scorePreloaded.resetOdometry(),
                scorePreloaded.cmd(),
                preloadToCoral.cmd(),
                coralToReef.cmd()
            ), routine)
        );

        return routine;
    }

    public Command cycleAfter(int cycles, Command base, AutoRoutine routine) {
        if (cycles <= 0) return base;

        AutoTrajectory reefToCoral = routine.trajectory("reef-to-coral-station");
        AutoTrajectory coralToReef = routine.trajectory("coral-station-to-reef");
        SequentialCommandGroup group = new SequentialCommandGroup(base);

        for (int i = 0; i < cycles; i++) {
            group.addCommands(reefToCoral.cmd(), coralToReef.cmd());
        }

        return group;
    }
}
