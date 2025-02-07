package frc.robot;

import java.util.function.Supplier;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIOSim;
import frc.robot.subsystems.coral.CoralIOTalonFX;
import frc.robot.subsystems.algae.AlgaeIntake;
import frc.robot.subsystems.algae.AlgaeIntakeIO;
import frc.robot.subsystems.algae.AlgaeIntakeIOSim;
import frc.robot.subsystems.cage.Cage;
import frc.robot.subsystems.cage.CageIO;
import frc.robot.subsystems.cage.CageIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

public class RobotContainer {
    private static RobotContainer instance;

    // Subsystems
    public final Drive drive;
    public final Vision vision;
    public final Elevator elevator;
    public final Coral coral;
    public final AlgaeIntake algae;
    public final Cage cage;

    // Auto
    private final Autonomous autonomous;

    // Visuals
    public final RobotVisualizer visualizer;

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final Supplier<Translation2d> joystickSupplier = () -> new Translation2d(
        driverController.getLeftY(), driverController.getLeftX());

    /* Controller trigger bindings */

    // Utilities
    public final Trigger xLock = driverController.x();
    public final Trigger resetGyro = driverController.start();
    public final Trigger resetElevator = operatorController.start();

    // Drive/point to different field POIs
    public final Trigger targetRight = driverController.rightBumper();
    public final Trigger targetLeft = driverController.leftBumper();
    public final Trigger targetCoralStation = driverController.a();

    // Run coral rollers to score and stow elevator
    public final Trigger score = driverController.rightTrigger(0.25);

    // Elevator setpoints
    public final Trigger l1 = operatorController.x();
    public final Trigger l2 = operatorController.a();
    public final Trigger l3 = operatorController.b();
    public final Trigger l4 = operatorController.y();
    public final Trigger stowManual = operatorController.leftBumper();

    // Manual subsystem controls
    public final Trigger elevatorManual = operatorController.leftTrigger(0.35);
    public final Trigger cageManual = operatorController.rightTrigger(0.35);
    public final Trigger coralManual = operatorController.rightBumper();

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        RobotContainer.instance = this;

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

            elevator = new Elevator(new ElevatorIOTalonFX());
            coral = new Coral(new CoralIOTalonFX());

            algae = new AlgaeIntake(new AlgaeIntakeIO() {});
            cage = new Cage(new CageIO() {});

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
                new VisionIO() {},
                new VisionIO() {}
            );

            elevator = new Elevator(new ElevatorIOSim());
            coral = new Coral(new CoralIOSim());

            algae = new AlgaeIntake(new AlgaeIntakeIOSim());
            cage = new Cage(new CageIOSim() {});

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

            elevator = new Elevator(new ElevatorIO() {});
            coral = new Coral(new CoralIO() {});

            algae = new AlgaeIntake(new AlgaeIntakeIO() {});
            cage = new Cage(new CageIO() {});

            break;
        }

        visualizer = new RobotVisualizer(elevator::getExtensionMeters, () -> Rotation2d.fromDegrees(45));

        // Setup autonomous features
        autonomous = new Autonomous(this);

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
                joystickSupplier,
                () -> driverController.getLeftTriggerAxis() > 0.25,
                () -> -driverController.getRightX()));

        // Switch to X pattern
        xLock.onTrue(Commands.runOnce(drive::xLock, drive));

        // Reset gyro to 0°
        resetGyro.onTrue(Commands.runOnce(() ->
            drive.setPose(new Pose2d(
                drive.getPose().getTranslation(),
                ChoreoAllianceFlipUtil.shouldFlip()
                    ? ChoreoAllianceFlipUtil.flip(new Rotation2d())
                    : new Rotation2d()
            )),
            drive).ignoringDisable(true));

        // Hold left trigger to enable elevator manual controls using the right stick.
        // This should be removed once elevator testing is complete
        elevatorManual.whileTrue(elevator.setVoltage(() ->
            MathUtil.applyDeadband(-operatorController.getRightY(), 0.1) * 8));
        elevatorManual.onFalse(elevator.setVoltage(() -> 0));

        // Hold right trigger to enable cage manual controls using the right stick.
        // This should be removed once cage testing is complete
        cageManual.whileTrue(cage.setVoltage(() ->
            MathUtil.applyDeadband(-operatorController.getRightY(), 0.1) * 4));
        cageManual.onFalse(cage.setVoltage(() -> 0));

        l1.onTrue(elevator.setPositionCommand(ElevatorConstants.l1Setpoint));
        l2.onTrue(elevator.setPositionCommand(ElevatorConstants.l2Setpoint));
        l3.onTrue(elevator.setPositionCommand(ElevatorConstants.l3Setpoint));
        l4.onTrue(elevator.setPositionCommand(ElevatorConstants.l4Setpoint));

        stowManual.onTrue(elevator.setPositionCommand(ElevatorConstants.coralStationSetpoint));

        score.onTrue(coral.setVoltageCommand(4));
        score.onFalse(coral.setVoltageCommand(0.0)
            .andThen(elevator.setPositionCommand(ElevatorConstants.coralStationSetpoint))
            .andThen(coral.intakeCommand()));

        resetElevator.onTrue(elevator.resetPositionCommand().ignoringDisable(true));

        targetLeft.whileTrue(DriveCommands.driveToNearestPole(drive, false, joystickSupplier));
        targetRight.whileTrue(DriveCommands.driveToNearestPole(drive, true, joystickSupplier));

        targetCoralStation.onTrue(coral.intakeCommand());
        // targetCoralStation.onTrue(DriveCommands.driveToCoralStation(drive).alongWith(coral.intakeCommand()));
        targetCoralStation.onFalse(drive.runOnce(() -> {}));

        coralManual.onTrue(coral.setVoltageCommand(4));
        coralManual.onFalse(coral.setVoltageCommand(0));

        // intakeAlgae.onTrue(algae.setRollerVoltage(12));
        // outtakeAlgae.onTrue(algae.setRollerVoltage(-12));
        // intakeAlgae.onFalse(algae.setRollerVoltage(0));
        // outtakeAlgae.onFalse(algae.setRollerVoltage(0));

        // intakeDown.onTrue(algae.setPivotPosition(90));
        // intakeDown.onFalse(algae.setPivotPosition(0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonomous.getSelectedAuto();
    }

    public Supplier<Translation2d> joystickMotionSupplier() {
        return joystickSupplier;
    }

    public static RobotContainer getInstance() { return instance; }
}
