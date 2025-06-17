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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOReal;
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
import frc.robot.util.FieldPOIs;
import frc.robot.util.RisingEdgeTrigger;
import frc.robot.autoalign.AutoAlign;
import frc.robot.autoalign.AutoAlign.Target;

public class RobotContainer {
    private static RobotContainer instance;

    // Subsystems
    public final Drive drive;
    public final Vision vision;
    public final Elevator elevator;
    public final Coral coral;
    public final AlgaeArm algae;
    public final Climber climber;

    // Auto
    public final Autonomous autonomous;

    // Visuals
    public final RobotVisualizer visualizer;

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final Supplier<Translation2d> joystickSupplier = () -> new Translation2d(driverController.getLeftY(), driverController.getLeftX());

    /* Triggers */

    public final Trigger hasNothing;

    // "Nothing" state
    public final RisingEdgeTrigger targetCoralStation;
    public final RisingEdgeTrigger algaeLow;
    public final RisingEdgeTrigger algaeHigh;
    public final Trigger autoIntake;

    // "Coral" state
    public final RisingEdgeTrigger targetRight;
    public final RisingEdgeTrigger targetLeft;
    public final RisingEdgeTrigger scoreCoral;

    // "Algae" state
    public final RisingEdgeTrigger targetNet;
    public final RisingEdgeTrigger targetProcessor;
    public final RisingEdgeTrigger scoreAlgae;

    // Elevator setpoints
    public final Trigger l1 = operatorController.x();
    public final Trigger l2 = operatorController.a();
    public final Trigger l3 = operatorController.b();
    public final Trigger l4 = operatorController.y();
    public final Trigger stowManual = operatorController.start();

    // Operator algae
    public final Trigger groundIntake = operatorController.leftBumper();
    public final Trigger lollipopIntake = operatorController.leftTrigger(0.25);
    public final Trigger dropAlgae = operatorController.rightTrigger(0.25);

    // Climber
    public final Trigger climberStow = operatorController.povLeft().or(operatorController.povRight());
    public final Trigger climberDeploy = operatorController.povUp();
    public final Trigger climberClimb = operatorController.povDown();

    // Manual subsystem controls
    public final Trigger elevatorManual = operatorController.rightTrigger(0.35);
    public final Trigger intakeManual = operatorController.rightBumper();

    // Utilities
    public final Trigger stopAll = driverController.y();
    public final Trigger resetGyro = driverController.start();

    // Operator utilities
    public final Trigger slowEject = operatorController.povRight();
    public final Trigger reverseRollers = operatorController.povLeft();
    public final Trigger elevatorApplyManual = operatorController.back();

    public final Command netPlaceCommand;
    public final Command intakeCommand;

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
            climber = new Climber(new ClimberIOReal());

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
            climber = new Climber(new ClimberIOSim() {});

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
            climber = new Climber(new ClimberIO() {});

            break;
        }

        visualizer = new RobotVisualizer(elevator::getExtensionMeters, () -> Rotation2d.fromDegrees(45));

        // Setup webcam streaming
        CameraServer.startAutomaticCapture();

        // Setup triggers
        hasNothing = coral.hasCoral.negate().and(algae.hasAlgae.negate());

        targetRight = new RisingEdgeTrigger(driverController.rightBumper(), coral.hasCoral);
        targetLeft = new RisingEdgeTrigger(driverController.leftBumper(), coral.hasCoral);
        scoreCoral = new RisingEdgeTrigger(driverController.rightTrigger(0.25), coral.hasCoral);
        autoIntake = coral.isUnused.and(algae.isUnused).and(hasNothing);

        targetNet = new RisingEdgeTrigger(driverController.leftBumper(), algae.hasAlgae);
        targetProcessor = new RisingEdgeTrigger(driverController.rightBumper(), algae.hasAlgae);
        scoreAlgae = new RisingEdgeTrigger(driverController.rightTrigger(0.25), algae.hasAlgae);

        targetCoralStation = new RisingEdgeTrigger(driverController.leftBumper(), hasNothing);
        algaeLow = new RisingEdgeTrigger(driverController.rightBumper(), hasNothing);
        algaeHigh = new RisingEdgeTrigger(driverController.rightTrigger(0.25), hasNothing);

        netPlaceCommand = new SequentialCommandGroup(
            algae.setPivotPosition(AlgaeConstants.stow),
            Commands.waitSeconds(0.35),
            coral.setVoltageCommand(3),
            Commands.waitSeconds(0.3),
            coral.setVoltageCommand(0),
            elevator.setPositionCommand(ElevatorConstants.stow)
        );

        intakeCommand = createIntakeCommand();

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
        BooleanSupplier slowMode = () -> driverController.getLeftTriggerAxis() > 0.25;

        // Scoring and utility triggers
        Trigger isTargetingNet = new Trigger(() -> AutoAlign.getTargetType().name().contains("NET"));
        RisingEdgeTrigger scoreNet = scoreAlgae.and(isTargetingNet);
        RisingEdgeTrigger scoreProcessor = scoreAlgae.and(isTargetingNet.negate()).and(new Trigger(() -> AutoAlign.getTargetType().name().contains("PROCESSOR")));

        RisingEdgeTrigger scoreL1 = scoreCoral.and(new Trigger(() -> elevator.getScheduledPosition() == ElevatorConstants.L1));
        RisingEdgeTrigger scoreLs234 = scoreCoral.and(new Trigger(() -> elevator.getScheduledPosition() == ElevatorConstants.L1));

        Trigger leaveReefZone = new Trigger(() -> drive.getPose().getTranslation().getDistance(
            AutoAlign.flipIfRed(FieldPOIs.REEF_CENTER).getTranslation()) > FieldPOIs.REEF_ZONE_DISTANCE);

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
        scoreLs234.onTrueOnFalse(
            coral.setVelocityCommand(() ->
                elevator.getTargetPosition() < ElevatorConstants.L4
                    ? CoralConstants.scoreRadPerSec : CoralConstants.l4RadPerSec),
            coral.setVoltageCommand(0)
                .andThen(algae.setPivotPosition(AlgaeConstants.stow))
                .andThen(elevator.setPositionCommand(ElevatorConstants.stow))
        );

        // Drive score sequence (L1)
        scoreL1.onTrueOnFalse(
            coral.setVelocityCommand(CoralConstants.l1RadPerSec)
                .andThen(Commands.waitSeconds(0.1))
                .andThen(elevator.setPositionCommand(ElevatorConstants.L2)),
            coral.setVoltageCommand(0)
                .andThen(elevator.setPositionCommand(ElevatorConstants.stow))
        );

        driverController.povDown().onTrue(coral.setVoltageCommand(-12));
        driverController.povDown().onFalse(coral.setVoltageCommand(0));

        // Driver coral auto align
        targetLeft.whileTrue(AutoAlign.autoAlignTo(Target.LEFT_BRANCH, this, joystickSupplier)
            .beforeStarting(() -> AutoAlign.setScheduledElevatorHeight(elevator.getScheduledPosition())));
        targetRight.whileTrue(AutoAlign.autoAlignTo(Target.RIGHT_BRANCH, this, joystickSupplier)
            .beforeStarting(() -> AutoAlign.setScheduledElevatorHeight(elevator.getScheduledPosition())));

        // Driver algae auto align
        algaeHigh.whileTrue(createAlgaeAlign(true));
        algaeLow.whileTrue(createAlgaeAlign(false));

        // Stow after grabbing algae and leaving reef zone
        leaveReefZone.and(RobotModeTriggers.autonomous().negate()).and(algae.hasAlgae).and(driverController.rightTrigger(0.25).negate())
            .onTrue(AutoAlign.clearTargetType()
            .andThen(elevator.setPositionCommand(ElevatorConstants.stow))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(coral.setVoltageCommand(CoralConstants.algaeHoldVoltage)));

        // Driver net auto align
        targetNet.whileTrue((AutoAlign.autoAlignTo(Target.NET, this, joystickSupplier))
            .beforeStarting(coral.setVelocityCommand(CoralConstants.algaeGrabRadPerSec)
                .andThen(Commands.runOnce(() -> AutoAlign.setScheduledElevatorHeight(ElevatorConstants.L4)))));

        // Driver score sequence (net)
        scoreNet.onTrue(netPlaceCommand);

        // Driver processor auto align
        targetProcessor.onTrue(AutoAlign.setTargetType(AutoAlign.Target.PROCESSOR)
            .andThen(coral.setVelocityCommand(CoralConstants.algaeGrabRadPerSec))
            .andThen(algae.setPivotPosition(AlgaeConstants.processor))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(coral.setVoltageCommand(CoralConstants.algaeHoldVoltage)));

        // Driver score sequence (processor)
        scoreProcessor.onTrue(coral.setVelocityCommand(140)
            .andThen(Commands.waitSeconds(0.5))
            .andThen(coral.setVoltageCommand(0)));

        // Intake + coral station align
        targetCoralStation.whileTrue(DriveCommands.alignToCoralStation(drive, joystickSupplier, slowMode));
        targetCoralStation.onTrue(intakeCommand);
        intakeManual.onTrue(intakeCommand);
        autoIntake.onTrue(intakeCommand);

        // Ground intake
        groundIntake.onTrue(algae.setPivotPosition(AlgaeConstants.groundIntake)
            .andThen(coral.setVelocityCommand(CoralConstants.algaeGrabRadPerSec)));
        groundIntake.onFalse(algae.setPivotPosition(AlgaeConstants.hold)
            .andThen(Commands.waitSeconds(1.0))
            .andThen(coral.setVoltageCommand(CoralConstants.algaeHoldVoltage)));

        // Ground intake
        lollipopIntake.onTrue(algae.setPivotPosition(AlgaeConstants.lollipopIntake)
            .andThen(coral.setVelocityCommand(CoralConstants.algaeGrabRadPerSec)));
        lollipopIntake.onFalse(algae.setPivotPosition(AlgaeConstants.hold)
            .andThen(Commands.waitSeconds(1.0))
            .andThen(coral.setVoltageCommand(CoralConstants.algaeHoldVoltage)));

        // Hold left trigger to enable elevator manual controls using the right stick.
        elevatorManual.whileTrue(elevator.setVoltage(() ->
            MathUtil.applyDeadband(-operatorController.getRightY(), 0.1) * 8));
        elevatorManual.onFalse(elevator.setPositionCommand(() -> elevator.getExtensionMeters()));

        // Schedule different reef heights
        l1.onTrue(elevator.schedulePositionCommand(ElevatorConstants.L1).andThen(AutoAlign.clearTargetType()));
        l2.onTrue(elevator.schedulePositionCommand(ElevatorConstants.L2).ignoringDisable(true));
        l3.onTrue(elevator.schedulePositionCommand(ElevatorConstants.L3).ignoringDisable(true));
        l4.onTrue(elevator.schedulePositionCommand(ElevatorConstants.L4).ignoringDisable(true));

        // Stow the elevator manually
        stowManual.onTrue(elevator.setPositionCommand(ElevatorConstants.stow)
            .andThen(coral.setVoltageCommand(0))
            .andThen(algae.setPivotPosition(AlgaeConstants.stow)));

        // Manually apply the elevator's scheduled position
        elevatorApplyManual.onTrue(elevator.applyScheduledPositionCommand());

        // Run the coral rollers slowly
        slowEject.onTrue(coral.setVoltageCommand(4.5));
        slowEject.onFalse(coral.setVoltageCommand(0.0));

        // Run the coral rollers slowly in reverse
        reverseRollers.onTrue(coral.setVoltageCommand(-4));
        reverseRollers.onFalse(coral.setVoltageCommand(0.0));

        // Climber deploy
        climberDeploy.onTrue(climber.setPivotPosition(ClimberConstants.deployPosition)
            .andThen(climber.setRollersVoltage(ClimberConstants.deployRollerVoltage)));

        // Climber stow
        climberStow.onTrue(climber.setPivotPosition(ClimberConstants.stowPosition)
            .andThen(climber.setRollersVoltage(0)));

        // Climber stow
        climberClimb.onTrue(climber.setPivotPosition(ClimberConstants.climbPosition)
            .andThen(climber.setRollersVoltage(ClimberConstants.climbRollerVoltage)));

        // Manual elevator pivot
        elevatorManual.whileTrue(climber.setPivotVoltage(() ->
            MathUtil.applyDeadband(-operatorController.getLeftY(), 0.1) * 10));
        elevatorManual.onFalse(climber.setPivotVoltage(() -> 0));
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

    public Command createAlgaeAlign(boolean high) {
        return AutoAlign.autoAlignTo(Target.ALGAE_ALIGN, this, joystickSupplier)
            .beforeStarting(() -> {
                // Schedule the proper elevator height
                AutoAlign.setScheduledElevatorHeight(high ? ElevatorConstants.algaeHigh : ElevatorConstants.algaeLow);
            })
            .until(AutoAlign.isAtTarget(Target.ALGAE_ALIGN, 0.1))
            .andThen(new ParallelCommandGroup(
                AutoAlign.autoAlignTo(Target.ALGAE_GRAB, this, joystickSupplier),
                algae.setPivotPosition(AlgaeConstants.grab)
                    .andThen(coral.setVelocityCommand(CoralConstants.algaeGrabRadPerSec))
            ))
            .deadlineFor(Commands.waitSeconds(0.25).andThen(algae.setPivotPosition(AlgaeConstants.grab)));

    }

    @Deprecated
    public void netTossWhen(Trigger trigger) {
        trigger.onTrue(new SequentialCommandGroup(
            elevator.setPositionCommand(ElevatorConstants.net),
            coral.setVelocityCommand(-90),
            Commands.waitSeconds(0.25),
            algae.setPivotPosition(AlgaeConstants.grab),
            Commands.waitSeconds(0.13),
            coral.setVoltageCommand(7),
            Commands.waitSeconds(0.37),
            algae.setPivotPosition(AlgaeConstants.stow),
            coral.setVoltageCommand(0),
            Commands.waitSeconds(0.2),
            elevator.setPositionCommand(ElevatorConstants.stow)
        ));
    }

    public Command createIntakeCommand() {
        Command armIntakeAssist = algae.setPivotPosition(AlgaeConstants.stow).asProxy()
            .andThen(Commands.waitSeconds(1.0))
            .andThen(algae.setPivotPosition(AlgaeConstants.intake).asProxy())
            .andThen(Commands.waitSeconds(0.25));
        Command intakeCommand = new SequentialCommandGroup(
            algae.setPivotPosition(AlgaeConstants.intake).asProxy(),
            coral.intakeUntilFunnelEnter(),
            coral.completeIntaking().deadlineFor(armIntakeAssist.repeatedly()),
            coral.setVoltageCommand(0)
        );
        return Commands.runOnce(() -> { if (intakeCommand.isScheduled()) intakeCommand.cancel(); }).andThen(intakeCommand);
    }
}
