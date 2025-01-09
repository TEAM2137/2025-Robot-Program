package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GameEvents;

public class Autonomous {
    public final AutoFactory factory;

    private final AutoChooser chooser;
    private final Drive drive;

    public Autonomous(RobotContainer robot) {
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
        SmartDashboard.putData(chooser);

        // Assign auto commands to autonomous trigger
        GameEvents.autonomous().whileTrue(chooser.selectedCommandScheduler());
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
        chooser.addRoutine("Example Auto", this::exampleAuto);
    }

    public AutoRoutine exampleAuto() {
        AutoRoutine routine = factory.newRoutine("pickupAndScore");

        // Load the routine's trajectories
        AutoTrajectory traj1 = routine.trajectory("test-trajectory");
        AutoTrajectory traj2 = routine.trajectory("test-trajectory");

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(
            Commands.sequence(
                traj1.resetOdometry(),
                traj1.cmd()
            )
        );

        // When the first trajectory ends, run the second one
        traj1.done().onTrue(traj2.cmd());

        return routine;
    }
}
