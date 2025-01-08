package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;


public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Elevator elevator;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Bindings
    public final Trigger resetGyro = controller.start();
    public final Trigger xLock = controller.x();
    public final Trigger rotationLock = controller.leftTrigger();

    public final Trigger l1 = controller.x();
    public final Trigger l2 = controller.a();
    public final Trigger l3 = controller.b();
    public final Trigger l4 = controller.y();
    public final Trigger coralStation = l1.or(l2).or(l3).or(l4);

    // Dashboard inputs
    // private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
        case REAL:
            // Real robot, instantiate hardware IO implementations
            drive = new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight)
            );

            vision = new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.cam0, drive::getRotation),
                new VisionIOLimelight(VisionConstants.cam1, drive::getRotation)
            );

            elevator = new Elevator(
                new ElevatorIOTalonFX()
            );

            break;

        case SIM:
            // Sim robot, instantiate physics sim IO implementations
            drive = new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight)
            );

            vision = new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(VisionConstants.cam0, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(VisionConstants.cam1, VisionConstants.robotToCamera1, drive::getPose)
            );

            elevator = new Elevator(
                new ElevatorIOSim()
            );

            break;

        default:
            // Replayed robot, disable IO implementations
            drive = new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {}
            );

            vision = new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {}
            );

            elevator = new Elevator(
                new ElevatorIO() {}
            );

            break;
        }

        // Set up auto routines
        // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        // autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        // autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        // autoChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> -controller.getRightX()));

        // Lock rotation to 0°
        rotationLock.whileTrue(DriveCommands.joystickDriveAtAngle(drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> new Rotation2d()));

        // Switch to X pattern
        xLock.onTrue(Commands.runOnce(drive::xLock, drive));

        // Reset gyro to 0°
        resetGyro.onTrue(Commands.runOnce(() ->drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),drive)
                .ignoringDisable(true));

        l1.onTrue(elevator.setPositionCommand(Elevator.Constants.L1));
        l2.onTrue(elevator.setPositionCommand(Elevator.Constants.L2));
        l3.onTrue(elevator.setPositionCommand(Elevator.Constants.L3));
        l4.onTrue(elevator.setPositionCommand(Elevator.Constants.L4));
        coralStation.whileFalse(elevator.setPositionCommand(Elevator.Constants.CORAL_STATION));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return autoChooser.get();
        return null;
    }
}
