package frc.robot;


import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.cameraserver.CameraServer;
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
import frc.robot.subsystems.coral.CoralConstants;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIOSim;
import frc.robot.subsystems.coral.CoralIOTalonFX;
import frc.robot.subsystems.algae.AlgaeArm;
import frc.robot.subsystems.algae.AlgaeArmIO;
import frc.robot.subsystems.algae.AlgaeArmIOSim;
import frc.robot.subsystems.algae.AlgaeArmIOTalonFX;
import frc.robot.subsystems.algae.AlgaeConstants;
import frc.robot.subsystems.cage.Cage;
import frc.robot.subsystems.cage.CageIO;
import frc.robot.subsystems.cage.CageIOSim;
import frc.robot.subsystems.cage.CageIOSparkMax;
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
import frc.robot.autoalign.AutoAlign;
import frc.robot.autoalign.AutoAlign.Target;
import frc.robot.util.FieldPOIs;

public class RobotContainer {
    private static RobotContainer instance;

    // Subsystems
    public final Drive drive;
    public final Vision vision;
    public final Elevator elevator;
    public final Coral coral;
    public final AlgaeArm algae;
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
    public final Trigger stopAll = driverController.y();
    public final Trigger resetGyro = driverController.start();
    public final Trigger resetElevator = operatorController.start();
    public final Trigger resetCage = operatorController.rightStick();
    public final Trigger resetAlgae = operatorController.leftStick();

    // Drive/point to different field POIs
    public final Trigger targetRight = driverController.rightBumper();
    public final Trigger targetLeft = driverController.leftBumper();
    public final Trigger targetAlgae = driverController.b();
    public final Trigger targetNet = driverController.x();
    public final Trigger targetCoralStation = driverController.a();

    // Run coral rollers to score and stow elevator
    public final Trigger score = driverController.rightTrigger(0.25);

    // Sequences for removing algae from the reef

    // Elevator setpoints
    public final Trigger l1 = operatorController.x();
    public final Trigger l2 = operatorController.a();
    public final Trigger l3 = operatorController.b();
    public final Trigger l4 = operatorController.y();
    public final Trigger stowManual = operatorController.leftBumper();

    // Manual subsystem controls
    public final Trigger elevatorManual = operatorController.leftTrigger(0.35);
    public final Trigger cageManual = operatorController.rightTrigger(0.35);
    public final Trigger algaeRollers = operatorController.rightBumper();
    public final Trigger slowEject = operatorController.povRight();
    public final Trigger reverseRollers = operatorController.povLeft();
    public final Trigger climberDeploy = operatorController.povUp();
    public final Trigger elevatorApplyManual = operatorController.back();

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

            algae = new AlgaeArm(new AlgaeArmIOTalonFX());
            cage = new Cage(new CageIOSparkMax());

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

            algae = new AlgaeArm(new AlgaeArmIOSim());
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

            algae = new AlgaeArm(new AlgaeArmIO() {});
            cage = new Cage(new CageIO() {});

            break;
        }

        visualizer = new RobotVisualizer(elevator::getExtensionMeters, () -> Rotation2d.fromDegrees(45));

        // Setup autonomous features
        autonomous = new Autonomous(this);

        // Configure the button bindings
        configureButtonBindings();

        // Setup webcam streaming
        CameraServer.startAutomaticCapture();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        BooleanSupplier slowMode = () -> driverController.getLeftTriggerAxis() > 0.25;

        // Scoring conditionals
        Trigger scoreAlgae = new Trigger(() -> FieldPOIs.ALGAE_LOCATIONS.contains(AutoAlign.flipIfRed(AutoAlign.getLastTargeted())));
        Trigger scoreL1 = scoreAlgae.negate().and(new Trigger(() -> elevator.getScheduledPosition() == ElevatorConstants.L1));
        Trigger scoreCoral = scoreAlgae.negate().and(scoreL1.negate());

        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive,
                () -> joystickSupplier.get(),
                slowMode, () -> -driverController.getRightX() * 0.75));

        // Stop all active subsystems
        stopAll.onTrue(coral.setVoltageCommand(0)
            .andThen(elevator.setVoltage(() -> 0)));

        // Reset gyro to 0°
        resetGyro.onTrue(Commands.runOnce(() ->
            drive.setPose(new Pose2d(
                drive.getPose().getTranslation(),
                ChoreoAllianceFlipUtil.shouldFlip()
                    ? ChoreoAllianceFlipUtil.flip(new Rotation2d())
                    : new Rotation2d()
            )),
            drive).ignoringDisable(true));

        // Driver score sequence (L2-L4)
        score.and(scoreCoral).onTrue(coral.setVelocityCommand(() ->
            elevator.getTargetPosition() < ElevatorConstants.L4
                ? CoralConstants.scoreRadPerSec : CoralConstants.l4RadPerSec));
        score.and(scoreCoral).onFalse(coral.setVoltageCommand(0)
            .andThen(elevator.setPositionCommand(ElevatorConstants.stow)));

        // Drive score sequence (L1)
        score.and(scoreL1).onTrue(coral.setVelocityCommand(CoralConstants.l1RadPerSec)
            .andThen(elevator.setPositionCommand(ElevatorConstants.L2)));
        score.and(scoreL1).onFalse(coral.setVoltageCommand(0)
            .andThen(elevator.setPositionCommand(ElevatorConstants.stow)));

        // Driver score sequence (remove algae)
        score.and(scoreAlgae).onTrue(algae.setPivotPosition(AlgaeConstants.deploy)
            .andThen(coral.setVoltageCommand(-6)));
        score.and(scoreAlgae).onFalse(algae.setPivotPosition(AlgaeConstants.stow)
            .andThen(AutoAlign.clearLastTargeted())
            .andThen(coral.setVoltageCommand(0))
            .andThen(Commands.waitSeconds(0.3))
            .andThen(elevator.setPositionCommand(ElevatorConstants.stow)));

        // Driver coral auto align
        targetLeft.whileTrue(AutoAlign.autoAlignTo(Target.LEFT_BRANCH, this, joystickSupplier)
            .beforeStarting(() -> AutoAlign.setScheduledElevatorHeight(elevator.getScheduledPosition())));
        targetRight.whileTrue(AutoAlign.autoAlignTo(Target.RIGHT_BRANCH, this, joystickSupplier)
            .beforeStarting(() -> AutoAlign.setScheduledElevatorHeight(elevator.getScheduledPosition())));

        // Driver algae auto align
        targetAlgae.whileTrue(AutoAlign.autoAlignTo(Target.ALGAE, this, joystickSupplier)
            .beforeStarting(() -> {
                // Schedule the proper elevator height
                int poseId = AutoAlign.getNewTargetPoseId(drive, Target.ALGAE, joystickSupplier);
                AutoAlign.setScheduledElevatorHeight(poseId % 2 == 0 ? ElevatorConstants.algaeHigh : ElevatorConstants.algaeLow);
            }));

        // Driver net auto align
        targetNet.whileTrue(AutoAlign.autoAlignTo(Target.NET, this, joystickSupplier)
            .beforeStarting(() -> AutoAlign.setScheduledElevatorHeight(ElevatorConstants.net)));

        // Driver coral station auto align
        targetCoralStation.onTrue(DriveCommands.alignToCoralStation(drive, joystickSupplier, slowMode));
        targetCoralStation.onTrue(Commands.runOnce(() -> algae.setPivotPositionRaw(AlgaeConstants.intake), coral)
            .andThen(coral.intakeUntilFunnelEnter())
            .andThen(Commands.runOnce(() -> algae.setPivotPositionRaw(AlgaeConstants.stow), coral))
            .andThen(coral.completeIntaking())
            .andThen(coral.setVoltageCommand(0)));
        targetCoralStation.onFalse(Commands.runOnce(() -> drive.getCurrentCommand().cancel(), drive));

        // Hold left trigger to enable elevator manual controls using the right stick.
        elevatorManual.whileTrue(elevator.setVoltage(() ->
            MathUtil.applyDeadband(-operatorController.getRightY(), 0.1) * 8));
        elevatorManual.onFalse(elevator.setPositionCommand(() -> elevator.getExtensionMeters()));

        // Hold right trigger to enable cage manual controls using the right stick.
        cageManual.whileTrue(cage.setVoltage(() ->
            MathUtil.applyDeadband(-operatorController.getRightY(), 0.1) * 12));
        cageManual.onFalse(cage.setVoltage(() -> 0));

        // Hold right trigger to enable algae arm manual controls using the left stick.
        cageManual.whileTrue(algae.setPivotVoltage(() ->
            MathUtil.applyDeadband(-operatorController.getLeftY(), 0.1) * -2.5));
        cageManual.onFalse(algae.targetCurrentPosition());

        // Schedule different reef heights. These commands cannot be run while targeting algae
        l1.and(targetAlgae.negate()).onTrue(elevator.schedulePositionCommand(ElevatorConstants.L1));
        l2.and(targetAlgae.negate()).onTrue(elevator.schedulePositionCommand(ElevatorConstants.L2).ignoringDisable(true));
        l3.and(targetAlgae.negate()).onTrue(elevator.schedulePositionCommand(ElevatorConstants.L3).ignoringDisable(true));
        l4.and(targetAlgae.negate()).onTrue(elevator.schedulePositionCommand(ElevatorConstants.L4).ignoringDisable(true));

        // Stow the elevator manually
        stowManual.onTrue(elevator.setPositionCommand(ElevatorConstants.stow)
            .andThen(algae.setPivotPosition(AlgaeConstants.stow)));

        // Reset the elevator encoder position
        resetElevator.onTrue(elevator.resetPositionCommand().ignoringDisable(true));
        resetAlgae.onTrue(algae.resetPositionCommand().ignoringDisable(true));
        resetCage.onTrue(cage.resetPositionCommand().ignoringDisable(true));

        // Manually run the algae arm rollers
        algaeRollers.onTrue(coral.setVoltageCommand(-6));
        algaeRollers.onFalse(coral.setVoltageCommand(0));

        // Manually apply the elevator's scheduled position
        elevatorApplyManual.onTrue(elevator.applyScheduledPositionCommand());

        slowEject.onTrue(coral.setVoltageCommand(4.5));
        slowEject.onFalse(coral.setVoltageCommand(0.0));

        reverseRollers.onTrue(coral.setVoltageCommand(-4));
        reverseRollers.onFalse(coral.setVoltageCommand(0.0));
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
