package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoAlignUtil;
import frc.robot.util.FieldPOIs;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import choreo.util.ChoreoAllianceFlipUtil;

public class DriveCommands {
    private static final double DEADBAND = 0.1;

    private static final double ANGLE_KP = 4.8;
    private static final double ANGLE_KD = 0.0;
    private static final double ANGLE_MAX_VELOCITY = 5.5;
    private static final double ANGLE_MAX_ACCELERATION = 40.0;

    private static final double DRIVE_KP = 2.8;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_MAX_VELOCITY = 3.0; // Meters/Sec
    private static final double DRIVE_MAX_ACCELERATION = 18.0; // Meters/Sec^2
    private static final double DRIVE_DEADBAND_METERS = 0.01; // For targeting

    private static final double ELEVATOR_RAISE_DISTANCE_METERS = 1.25; // For targeting

    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private static final double WHEEL_RADIUS_MAX_VELOCITY = 1.0; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.5; // Rad/Sec^2

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            Supplier<Translation2d> movementSupplier,
            BooleanSupplier slowMode,
            DoubleSupplier omegaSupplier) {
        return Commands.run(() -> {
            // Get linear velocity
            Translation2d movementRaw = movementSupplier.get();
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(-movementRaw.getX(), -movementRaw.getY());

            // Apply rotation deadband
            double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

            // Square rotation value for more precise control
            omega = Math.copySign(omega * omega, omega);

            double multiplier = slowMode.getAsBoolean() ? 0.3 : 1.0;

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds = new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                omega * drive.getMaxAngularSpeedRadPerSec()
            );
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()));
        }, drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            Supplier<Translation2d> movementSupplier,
            BooleanSupplier slowMode,
            Supplier<Rotation2d> rotationSupplier) {
        // Create PID controller
        ProfiledPIDController angleController =
            new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(() -> {
            // Get linear velocity
            Translation2d movementRaw = movementSupplier.get();
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(-movementRaw.getX(), -movementRaw.getY());

            // Calculate angular speed
            double omega = angleController.calculate(
                drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

            double multiplier = slowMode.getAsBoolean() ? 0.3 : 1.0;

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds = new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                omega
            );
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation())
            );
        }, drive)
        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(
            drive.getRotation().getRadians(),
            drive.getAngularSpeedRadsPerSec()), drive);
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(() -> {
                velocitySamples.clear();
                voltageSamples.clear();
            }),

            // Allow modules to orient
            Commands.run(() -> {
                drive.runCharacterization(0.0);
            }, drive).withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(() -> {
                double voltage = timer.get() * FF_RAMP_RATE;
                drive.runCharacterization(voltage);
                velocitySamples.add(drive.getFFCharacterizationVelocity());
                voltageSamples.add(voltage);
            }, drive)
            // When cancelled, calculate and print results
            .finallyDo(() -> {
                int n = velocitySamples.size();
                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;
                for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                }
                double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                NumberFormat formatter = new DecimalFormat("#0.00000");
                System.out.println("********** Drive FF Characterization Results **********");
                System.out.println("\tkS: " + formatter.format(kS));
                System.out.println("\tkV: " + formatter.format(kV));
            })
        );
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> limiter.reset(0.0)),

                // Turn in place, accelerating up to full speed
                Commands.run(() -> {
                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                }, drive)
            ),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(() -> {
                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = drive.getRotation();
                    state.gyroDelta = 0.0;
                }),

                // Update gyro delta
                Commands.run(() -> {
                    var rotation = drive.getRotation();
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;
                })
                // When cancelled, calculate and print results
                .finallyDo(() -> {
                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                    double wheelDelta = 0.0;
                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    }
                    double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                    NumberFormat formatter = new DecimalFormat("#0.000");
                    System.out.println("********** Wheel Radius Characterization Results **********");
                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                    System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                    System.out.println("\tWheel Radius: "
                            + formatter.format(wheelRadius)
                            + " meters, "
                            + formatter.format(Units.metersToInches(wheelRadius))
                            + " inches");
                })
            )
        );
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    private static Pose2d target;
    private static Pose2d lastTargeted = new Pose2d();

    public static Pose2d getActiveTarget() {
        return target;
    }

    public static Pose2d getLastTargeted() {
        return lastTargeted;
    }

    public static Command autoAlignTo(AutoAlignUtil.Target targetType, RobotContainer robot, Supplier<Translation2d> motionSupplier) {
        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
            ANGLE_KP, 0.0, ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION)
        );
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.sequence(
            Commands.runOnce(() -> {
                target = getNewTargetPose(robot.drive, targetType, motionSupplier);
                lastTargeted = target;
            }),
            driveToTargetCommand(robot.drive, angleController).alongWith(Commands.run(() -> {
                Translation2d robotTranslation = robot.drive.getPose().getTranslation();
                if (target.getTranslation().getDistance(robotTranslation) < ELEVATOR_RAISE_DISTANCE_METERS) {
                    robot.elevator.applyScheduledPosition();
                }
            }))
        );
    }

    public static Command driveToCoralStation(Drive drive, Supplier<Translation2d> joystickSupplier, BooleanSupplier slowMode) {
        // Construct command
        return joystickDriveAtAngle(drive, joystickSupplier, slowMode, () -> {
            return (drive.getPose().getY() < 8.19912 / 2.0 == !ChoreoAllianceFlipUtil.shouldFlip())
                    ? AutoAlignUtil.flipIfRed(FieldPOIs.CORAL_STATION_BOTTOM).getRotation()
                    : AutoAlignUtil.flipIfRed(FieldPOIs.CORAL_STATION_TOP).getRotation();
        });
    }

    public static Pose2d getNewTargetPose(Drive drive, AutoAlignUtil.Target targetType, Supplier<Translation2d> motionSupplier) {
        return AutoAlignUtil.flipIfRed(AutoAlignUtil.fromPoseId(getNewTargetPoseId(drive, targetType, motionSupplier), targetType));
    }

    public static int getNewTargetPoseId(Drive drive, AutoAlignUtil.Target targetType, Supplier<Translation2d> motionSupplier) {
        Pose2d pose = drive.getPose();
        Translation2d motionVector = new Translation2d();

        // Ignore transformations if no joystick values are inputted
        if (motionSupplier.get().getNorm() > 0.1) {
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

            // Convert joystick inputs to robot relative (otherwise robot rotation messes with targeting)
            motionVector = AutoAlignUtil.normalize(
                motionSupplier.get().rotateBy((isFlipped
                    ? pose.getRotation().plus(new Rotation2d(Math.PI))
                    : pose.getRotation()).unaryMinus())
            );
        }

        // Find the correct pole to target
        return AutoAlignUtil.mapToPoseId(targetType, drive, motionVector);
    }

    private static Command driveToTargetCommand(Drive drive, ProfiledPIDController angleController) {
        // Create drive PID controller
        ProfiledPIDController driveController = new ProfiledPIDController(
            DRIVE_KP, 0.0, DRIVE_KD,
            new TrapezoidProfile.Constraints(DRIVE_MAX_VELOCITY, DRIVE_MAX_ACCELERATION)
        );

        // Run the command
        return Commands.runEnd(() -> {
            // Dynamically calculate drive constraints based on elevator height
            double elevatorHeight = RobotContainer.getInstance().elevator.getExtensionMeters();
            double velocityScaling = 1 - (elevatorHeight / 3.0);
            double accelScaling = 1 - (elevatorHeight / 6.0);
            TrapezoidProfile.Constraints driveConstraints = new TrapezoidProfile.Constraints(
                DRIVE_MAX_VELOCITY * velocityScaling,
                DRIVE_MAX_ACCELERATION * accelScaling);

            Translation2d toTarget = new Translation2d(
                drive.getPose().getX() - target.getX(),
                drive.getPose().getY() - target.getY()
            );
            Translation2d normalized = new Translation2d(
                driveController.calculate(MathUtil.applyDeadband(toTarget.getNorm(), DRIVE_DEADBAND_METERS),
                    new TrapezoidProfile.State(), driveConstraints),
                toTarget.getAngle()
            );

            // Calculate angular speed
            double omega = angleController.calculate(drive.getRotation().getRadians(),
                target.getRotation().getRadians());

            // Check if it's red alliance
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

            // Convert to field relative speeds
            ChassisSpeeds speeds = new ChassisSpeeds(
                normalized.getX() * (isFlipped ? -1 : 1),
                normalized.getY() * (isFlipped ? -1 : 1),
                omega
            );

            // Drive the robot to targets
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()
            ));
        }, () -> target = null, drive)
        .beforeStarting(() -> {
            // Reset pid controllers
            double targetDistance = drive.getPose().getTranslation().getDistance(target.getTranslation());
            driveController.reset(
                targetDistance,
                // Calculate the robots starting velocity towards the target
                drive.getLinearSpeedMetersPerSec() * AutoAlignUtil.dot(
                    AutoAlignUtil.normalize(drive.getLinearSpeedsVector()),
                    AutoAlignUtil.normalize(drive.getPose().getTranslation().minus(target.getTranslation())))
            );
            driveController.calculate(targetDistance);
            angleController.reset(
                drive.getRotation().getRadians(),
                drive.getAngularSpeedRadsPerSec()
            );
        });
    }
}
