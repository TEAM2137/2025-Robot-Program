package frc.robot;


import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.autoalign.AutoAlignCommand;
import frc.robot.autoalign.TargetSelector;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.autoalign.LegacyAutoAlign;
import frc.robot.autoalign.LegacyAutoAlign.Target;

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
    public final LoggedNetworkBoolean shouldManualClimb = new LoggedNetworkBoolean("ManualClimb", false);

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final Supplier<Translation2d> joystickSupplier = () -> new Translation2d(driverController.getLeftY(), driverController.getLeftX());

    /* Triggers */

    public final Trigger hasNothing;

    // "Nothing" state
    public final RisingEdgeTrigger targetCoralStation;
    public final RisingEdgeTrigger algaeGrab;
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
    public final RisingEdgeTrigger groundIntake;
    public final RisingEdgeTrigger lollipopIntake;
    public final Trigger dropAlgae = operatorController.rightTrigger(0.25);

    // Climber
    public final Trigger climberStow = operatorController.povLeft().or(operatorController.povRight());
    public final Trigger climberDeploy = operatorController.povUp();
    public final Trigger climberClimb = operatorController.povDown();

    // Manual subsystem controls
    public final Trigger elevatorManual = operatorController.back();
    public final Trigger climbManual = new Trigger(shouldManualClimb::get);

    // Utilities
    public final Trigger stopAll = driverController.y();
    public final Trigger resetGyro = driverController.start();

    public final Command netPlaceCommand;
    public final Command intakeCommand;

    // Leave Reef Zone
    public final Trigger leaveReefZone;

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
        // CameraServer.startAutomaticCapture();

        // Setup triggers
        hasNothing = coral.hasCoral.negate().and(algae.hasAlgae.negate());

        leaveReefZone = new Trigger(() -> drive.getPose().getTranslation().getDistance(
                TargetSelector.flipIfRed(FieldPOIs.REEF_CENTER).getTranslation()) > FieldPOIs.REEF_ZONE_DISTANCE);

        targetRight = new RisingEdgeTrigger(driverController.rightBumper(), coral.hasCoral, true);
        targetLeft = new RisingEdgeTrigger(driverController.leftBumper(), coral.hasCoral, true);
        scoreCoral = new RisingEdgeTrigger(driverController.rightTrigger(0.25), coral.hasCoral);
        autoIntake = coral.isUnused.and(algae.isUnused).and(elevator.isUnused).and(hasNothing).and(leaveReefZone).and(RobotModeTriggers.teleop());

        targetNet = new RisingEdgeTrigger(driverController.leftBumper(), algae.hasAlgae);
        targetProcessor = new RisingEdgeTrigger(driverController.rightBumper(), algae.hasAlgae);
        scoreAlgae = new RisingEdgeTrigger(driverController.rightTrigger(0.25), algae.hasAlgae);

        targetCoralStation = new RisingEdgeTrigger(driverController.leftBumper(), hasNothing);
        algaeGrab = new RisingEdgeTrigger(driverController.rightBumper(), hasNothing);
        groundIntake = new RisingEdgeTrigger(operatorController.leftTrigger(0.25), hasNothing);
        lollipopIntake = new RisingEdgeTrigger(operatorController.leftBumper(), hasNothing);

        netPlaceCommand = new SequentialCommandGroup(
            coral.setVoltageCommand(CoralConstants.algaeNetScore),
            Commands.waitSeconds(0.3),
            coral.setVoltageCommand(0),
            algae.setPivotPosition(AlgaeConstants.hold),
            Commands.waitSeconds(0.3),
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
        Trigger isTargetingNet = new Trigger(() -> LegacyAutoAlign.getTargetType().name().contains("NET"));
        Trigger isL1Selected = new Trigger(() -> elevator.getScheduledPosition() == ElevatorConstants.L1);
        Trigger isAtL4Height = new Trigger(() -> elevator.isAtTarget() && elevator.getTargetPosition() == ElevatorConstants.L4);
        Trigger enterReefZone = leaveReefZone.negate();

        RisingEdgeTrigger scoreNet = scoreAlgae.and(isTargetingNet);
        RisingEdgeTrigger scoreL1 = scoreCoral.and(isL1Selected);
        RisingEdgeTrigger scoreLs234 = scoreCoral.and(isL1Selected.negate());

        // test auto align command (remove at some point)

        AutoAlignCommand testAlign = AutoAlignCommand.builder()
                .withTargetPose(new Pose2d(2.0, 4.0, Rotation2d.kZero))
                .withFinalVelocity(new Translation2d(0.0, 0.0))
                .withEndTolerance(0.5)
                .build();

        AutoAlignCommand testAlign2 = AutoAlignCommand.builder()
                .withTargetPose(new Pose2d(0.0, 4.0, Rotation2d.kZero))
                .build();

        driverController.a().whileTrue(testAlign.andThen(testAlign2));

        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, joystickSupplier,
                slowMode, () -> -driverController.getRightX() * 0.75).withName("Default Drive"));

        // Stop all active subsystems
        stopAll.toggleOnTrue(coral.setVoltageCommand(0)
            .andThen(climber.setRollersVoltage(0))
            .andThen(elevator.setVoltage(() -> 0)).withName("Stop all subsystems"));

        // Reset gyro to 0°
        resetGyro.onTrue(Commands.runOnce(() ->
            drive.setPose(new Pose2d(
                drive.getPose().getTranslation(),
                ChoreoAllianceFlipUtil.shouldFlip()
                    ? ChoreoAllianceFlipUtil.flip(new Rotation2d())
                    : new Rotation2d()
            )),
            drive).ignoringDisable(true).withName("Reset Gyro"));

        // Driver score sequence (L2-L4)
        scoreLs234.onTrue(coral.setVelocityCommand(() -> elevator.getTargetPosition() < ElevatorConstants.L4
                ? CoralConstants.scoreRadPerSec : CoralConstants.l4RadPerSec).repeatedly()
            .until(coral.hasCoral.negate())
            .andThen(new WaitCommand(0.15))
            .andThen(coral.setVoltageCommand(0))
            .andThen(algae.setPivotPosition(AlgaeConstants.stow))
            .andThen(elevator.setPositionCommand(ElevatorConstants.stow)).withName("L2-L4 Score"));

        // Drive score sequence (L1)
        scoreL1.whileTrue(coral.setVelocityCommand(CoralConstants.l1RadPerSec)
            .andThen(Commands.waitSeconds(0.1))
            .andThen(elevator.setPositionCommand(ElevatorConstants.L2)).repeatedly()
            .finallyDo(() -> elevator.setPositionCommand(ElevatorConstants.stow)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(coral.setVoltageCommand(0))
                .withName("L1 Score B").schedule())
            .withName("L1 score A"));

        driverController.povDown().onTrue(coral.setVoltageCommand(-12).withName("Manual Coral"));
        driverController.povDown().onFalse(coral.setVoltageCommand(0).withName("Manual Coral Stop"));

        AutoAlignCommand alignLeft = AutoAlignCommand.builder()
                .withTargetSelector(TargetSelector.LEFT_BRANCHES)
                //.runCommandAtDistance(1.0, elevator.applyScheduledPositionCommand())
                .build("Align to Left Branch");

        AutoAlignCommand alignRight = AutoAlignCommand.builder()
                .withTargetSelector(TargetSelector.RIGHT_BRANCHES)
                //.runCommandAtDistance(1.0, elevator.applyScheduledPositionCommand())
                .build("Align to Right Branch");

        // Driver coral auto align
        targetLeft.whileTrue(alignLeft);
        targetRight.whileTrue(alignRight);

        // Driver algae auto align
        algaeGrab.whileTrue(createAlgaeAlign(() -> LegacyAutoAlign.getNewTargetPoseId(
                drive, Target.ALGAE_ALIGN, joystickSupplier) % 2 == 0)
            .withName("Algae Grab"));

        // Stow after grabbing algae and leaving reef zone
        leaveReefZone.and(RobotModeTriggers.autonomous().negate())
                .and(algae.hasAlgae).and(isTargetingNet.negate())
                .and(driverController.rightTrigger(0.25).negate())
            .onTrue(LegacyAutoAlign.clearTargetType()
            .andThen(elevator.setPositionCommand(ElevatorConstants.stow))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(coral.setVoltageCommand(CoralConstants.algaeHoldVoltage))
            .andThen(algae.setPivotPosition(AlgaeConstants.hold)).withName("Stow Algae after Leaving Reef"));

        enterReefZone.and(RobotModeTriggers.autonomous().negate())
                .and(coral.hasCoral).and(isL1Selected)
                .and(driverController.rightTrigger(0.25).negate())
            .onTrue(elevator.applyScheduledPositionCommand()
            .withName("Apply L1 height"));

        // Driver net auto align
        targetNet.whileTrue(DriveCommands.joystickDriveAtAngle(drive, joystickSupplier, slowMode, () ->
                (drive.getPose().getX() > 8.77) ? Rotation2d.kZero : Rotation2d.k180deg)
            .alongWith(Commands.run(() -> {
                if (drive.getPose().getX() > 7 && drive.getPose().getX() < 10.5)
                    elevator.setPosition(ElevatorConstants.L4);
            }, elevator, algae))
            .beforeStarting(LegacyAutoAlign.setTargetType(Target.NET))
            .withName("Target Net"));

        isAtL4Height.and(algae.hasAlgae).onTrue(algae.setPivotPosition(AlgaeConstants.stow));

        // Driver score sequence (net)
        scoreNet.onTrue(netPlaceCommand.withName("Place in Net"));

        // Driver processor auto align
        targetProcessor.onTrue(LegacyAutoAlign.setTargetType(LegacyAutoAlign.Target.PROCESSOR)
            .andThen(coral.setVelocityCommand(CoralConstants.algaeGrabRadPerSec))
            .andThen(algae.setPivotPosition(AlgaeConstants.processor))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(coral.setVoltageCommand(CoralConstants.algaeHoldVoltage))
            .andThen(Commands.waitUntil(driverController.rightTrigger(0.25)))
            .andThen(coral.setVelocityCommand(140)
            .andThen(Commands.waitSeconds(0.5))
            .andThen(coral.setVoltageCommand(0)))
            .andThen(Commands.waitUntil(driverController.rightBumper().negate().and(driverController.rightTrigger(0.25).negate())))
            .withName("Target Processor"));

        // Driver score sequence (processor)
        // scoreProcessor.onTrue(coral.setVelocityCommand(140)
        //     .andThen(Commands.waitSeconds(0.5))
        //     .andThen(coral.setVoltageCommand(0))
        //     .withName("Score in Processor"));

        // Intake + coral station align
        targetCoralStation.whileTrue(DriveCommands.alignToCoralStation(drive, joystickSupplier, slowMode)
            .withName("Align to Coral Station"));
        autoIntake.onTrue(elevator.setPositionCommand(ElevatorConstants.stow).alongWith(intakeCommand)
            .withName("Auto intake"));

        // Ground intake
        groundIntake.onTrue(algae.setPivotPosition(AlgaeConstants.groundIntake)
            .andThen(coral.setVelocityCommand(CoralConstants.algaeGrabRadPerSec)).repeatedly()
            .withName("Ground Algae Intake onTrue"));
        groundIntake.onFalse(Commands.waitSeconds(0.2)
            .andThen(algae.setPivotPosition(AlgaeConstants.hold))
            .andThen(Commands.waitSeconds(1.0))
            .andThen(coral.setVoltageCommand(CoralConstants.algaeHoldVoltage))
            .withName("Ground Algae Intake onFalse"));

        // Ground intake
        lollipopIntake.onTrue(algae.setPivotPosition(AlgaeConstants.lollipopIntake)
            .andThen(coral.setVelocityCommand(CoralConstants.algaeGrabRadPerSec).repeatedly())
            .withName("Lollipop Algae Intake onTrue"));
        lollipopIntake.onFalse(Commands.waitSeconds(0.2)
            .andThen(algae.setPivotPosition(AlgaeConstants.hold))
            .andThen(Commands.waitSeconds(1.0))
            .andThen(coral.setVoltageCommand(CoralConstants.algaeHoldVoltage))
            .withName("Lollipop Algae Intake onFalse"));

        dropAlgae.onTrue(coral.setVoltageCommand(6).withName("Drop Algae onTrue"));
        dropAlgae.onFalse(coral.setVoltageCommand(0).withName("Drop Algae onFalse"));

        // Hold left trigger to enable elevator manual controls using the right stick.
        elevatorManual.whileTrue(elevator.setVoltage(() ->
            MathUtil.applyDeadband(-operatorController.getRightY(), 0.1) * 8)
                .withName("Manual Elevator"));
        elevatorManual.onFalse(elevator.setPositionCommand(elevator::getExtensionMeters));
        elevatorManual.and(operatorController.rightStick()).onTrue(elevator.resetPositionCommand().ignoringDisable(true));

        // Schedule different reef heights
        l1.onTrue(elevator.schedulePositionCommand(ElevatorConstants.L1).andThen(LegacyAutoAlign.clearTargetType()));
        l2.onTrue(elevator.schedulePositionCommand(ElevatorConstants.L2).ignoringDisable(true));
        l3.onTrue(elevator.schedulePositionCommand(ElevatorConstants.L3).ignoringDisable(true));
        l4.onTrue(elevator.schedulePositionCommand(ElevatorConstants.L4).ignoringDisable(true));

        // Stow the elevator manually
        stowManual.onTrue(elevator.setPositionCommand(ElevatorConstants.stow)
            .andThen(coral.setVoltageCommand(0))
            .andThen(algae.setPivotPosition(AlgaeConstants.stow))
            .withName("Manual Stow"));

        // Run the coral rollers slowly
        // slowEject.onTrue(coral.setVoltageCommand(4.5).withName("Slow Coral Eject"));
        // slowEject.onFalse(coral.setVoltageCommand(0.0));

        // Run the coral rollers slowly in reverse
        // reverseRollers.onTrue(coral.setVoltageCommand(-4).withName("Reverse Coral Eject"));
        // reverseRollers.onFalse(coral.setVoltageCommand(0.0));

        // Climber deploy
        climberDeploy.onTrue(climber.setPivotPosition(ClimberConstants.deployPosition)
            .andThen(climber.setRollersVoltage(ClimberConstants.deployRollerVoltage))
            .withName("Climber Deploy"));

        // Climber stow
        climberStow.onTrue(climber.setPivotPosition(ClimberConstants.stowPosition)
            .andThen(climber.setRollersVoltage(0))
            .withName("Climber Stow"));

        // Climber climb
        climberClimb.whileTrue(climber.setPivotPosition(ClimberConstants.climbPosition)
            .andThen(climber.setRollersVoltage(ClimberConstants.climbRollerVoltage))
            .andThen(algae.setPivotPosition(AlgaeConstants.hold))
            .andThen(coral.setVoltageCommand(0)).repeatedly()
            .withName("Climber Climb"));
        climberClimb.onFalse(coral.setVoltageCommand(0)
            .andThen(algae.setPivotPosition(AlgaeConstants.hold))
            .andThen(climber.setPivotVoltage(() -> 0)));

        // Manual elevator pivot
        climbManual.whileTrue(climber.setPivotVoltage(() -> MathUtil.applyDeadband(-operatorController.getLeftY(), 0.1) * 6)
            .ignoringDisable(true).withName("Climb Manual"));
        climbManual.onFalse(climber.setPivotVoltage(() -> 0).ignoringDisable(true));

        // Zero climber
        climbManual.and(operatorController.leftStick()).onTrue(climber.resetPositionCommand().ignoringDisable(true));

        // Debug RisingEdgeTriggers
        scoreCoral.onTrue(new InstantCommand(() -> Logger.recordOutput("Triggers/scoreCoral", true)));
        scoreCoral.onFalse(new InstantCommand(() -> Logger.recordOutput("Triggers/scoreCoral", false)));

        scoreLs234.onTrue(new InstantCommand(() -> Logger.recordOutput("Triggers/scoreLs234", true)));
        scoreLs234.onFalse(new InstantCommand(() -> Logger.recordOutput("Triggers/scoreLs234", false)));
    }

    public void logTriggers() {
        Logger.recordOutput("Triggers/hasNothing", hasNothing.getAsBoolean());
        Logger.recordOutput("Triggers/hasCoral", coral.hasCoral.getAsBoolean());
        Logger.recordOutput("Triggers/hasAlgae", algae.hasAlgae.getAsBoolean());

        Logger.recordOutput("Triggers/coralUnused", coral.isUnused.getAsBoolean());
        Logger.recordOutput("Triggers/algaeUnused", algae.isUnused.getAsBoolean());

        var coralCommand = coral.getCurrentCommand();
        if (coralCommand != null) Logger.recordOutput("Triggers/CoralCommand", coralCommand.getName());
        else Logger.recordOutput("Triggers/CoralCommand", "No Command");

        Logger.recordOutput("Triggers/AutoAlignAtTarget", LegacyAutoAlign.isAtTarget(LegacyAutoAlign.getTargetType(), 0.1, 2.0));
    }

    public Supplier<Translation2d> joystickMotionSupplier() {
        return joystickSupplier;
    }

    public static RobotContainer getInstance() { return instance; }

    public Command createAlgaeAlign(BooleanSupplier high) {
        return LegacyAutoAlign.autoAlignTo(Target.ALGAE_ALIGN, this, joystickSupplier)
            .beforeStarting(() -> {
                // Schedule the proper elevator height
                LegacyAutoAlign.setScheduledElevatorHeight(high.getAsBoolean() ? ElevatorConstants.algaeHigh : ElevatorConstants.algaeLow);
            })
            .deadlineFor(Commands.waitSeconds(0.25).andThen(algae.setPivotPosition(AlgaeConstants.grab)))
            .until(new Trigger(LegacyAutoAlign.isAtTarget(Target.ALGAE_ALIGN,
                0.1, 2.0)).and(algae::isAtTarget))
            .andThen(new ParallelCommandGroup(
                LegacyAutoAlign.autoAlignTo(Target.ALGAE_GRAB, this, joystickSupplier),
                algae.setPivotPosition(AlgaeConstants.grab)
                    .andThen(coral.setVelocityCommand(CoralConstants.algaeGrabRadPerSec))
            )).finallyDo(() -> algae.setPivotPosition(AlgaeConstants.hold).schedule());
    }

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
            coral.setVoltageCommand(0),
            algae.setPivotPosition(AlgaeConstants.stow).asProxy()
        );
        return Commands.runOnce(() -> { if (intakeCommand.isScheduled()) intakeCommand.cancel(); }).andThen(intakeCommand).withName("Intake");
    }
}
