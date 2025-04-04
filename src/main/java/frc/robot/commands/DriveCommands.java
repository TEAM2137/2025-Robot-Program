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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.autoalign.AutoAlign;
import frc.robot.subsystems.drive.Drive;
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
    public static final double DEADBAND = 0.1;

    public static final double ANGLE_KP = 4.5;
    public static final double ANGLE_KD = 0.2;
    public static final double ANGLE_MAX_VELOCITY = 5.5;
    public static final double ANGLE_MAX_ACCELERATION = 45.0;
    public static final double ANGLE_DEADBAND = 0.0045;

    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private static final double WHEEL_RADIUS_MAX_VELOCITY = 1.0; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.5; // Rad/Sec^2

    private static final double DRIVE_MAX_ACCELERATION = 20.0; // Meters/Sec^2

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

            double elevatorHeight = RobotContainer.getInstance().elevator.getExtensionMeters();
            double accelScaling = MathUtil.clamp(1 - (elevatorHeight / 1.8), 0.2, 1);
            double velocityScaling = MathUtil.clamp(1 - (elevatorHeight / 2.5), 0.2, 1);

            Translation2d finalVelocity = limitAccelerationFor(
                drive.getLinearSpeedsVector(),
                new Translation2d(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * multiplier
                ).times(velocityScaling),
                DRIVE_MAX_ACCELERATION * accelScaling
            );

            finalVelocityPublisher.accept(finalVelocity);

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds = new ChassisSpeeds(
                finalVelocity.getX(), finalVelocity.getY(),
                omega * drive.getMaxAngularSpeedRadPerSec() * velocityScaling
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
            double omega = angleController.calculate(drive.getRotation().getRadians(), rotationSupplier.get().getRadians());
            if (Math.abs(angleController.getPositionError()) < ANGLE_DEADBAND) omega = 0.0;

            double multiplier = slowMode.getAsBoolean() ? 0.3 : 1.0;

            double elevatorHeight = RobotContainer.getInstance().elevator.getExtensionMeters();
            double accelScaling = MathUtil.clamp(1 - (elevatorHeight / 2.0), 0, 1);
            double velocityScaling = MathUtil.clamp(1 - (elevatorHeight / 3.5), 0, 1);

            Translation2d finalVelocity = limitAccelerationFor(
                drive.getLinearSpeedsVector(),
                new Translation2d(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * multiplier
                ).times(velocityScaling),
                DRIVE_MAX_ACCELERATION * accelScaling
            );

            finalVelocityPublisher.accept(finalVelocity);

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds = new ChassisSpeeds(
                finalVelocity.getX(), finalVelocity.getY(),
                omega * velocityScaling
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

    public static ProfiledPIDController getAngleController() {
        return new ProfiledPIDController(
            ANGLE_KP, 0.0, ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION)
        );
    }

    public static Command alignToCoralStation(Drive drive, Supplier<Translation2d> joystickSupplier, BooleanSupplier slowMode) {
        // Construct command
        return joystickDriveAtAngle(drive, joystickSupplier, slowMode, () -> {
            return (drive.getPose().getY() < 8.19912 / 2.0 == !ChoreoAllianceFlipUtil.shouldFlip())
                    ? AutoAlign.flipIfRed(FieldPOIs.CORAL_STATION_BOTTOM).getRotation()
                    : AutoAlign.flipIfRed(FieldPOIs.CORAL_STATION_TOP).getRotation();
        });
    }

    private static StructPublisher<Translation2d> currentVelocityPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("AccelLimiting/CurrentVelocity", Translation2d.struct).publish();
        private static StructPublisher<Translation2d> finalVelocityPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("AccelLimiting/FinalVelocity", Translation2d.struct).publish();
    private static StructPublisher<Translation2d> wantedVelocityPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("AccelLimiting/WantedVelocity", Translation2d.struct).publish();

    public static Translation2d limitAccelerationFor(Translation2d currentVelocity, Translation2d wantedVelocity, double maxAcceleration) {
        currentVelocityPublisher.accept(currentVelocity);
        wantedVelocityPublisher.accept(wantedVelocity);

        Translation2d flippedVelocity = ChoreoAllianceFlipUtil.shouldFlip() ? new Translation2d().minus(currentVelocity) : currentVelocity;
        Translation2d desiredAccel = wantedVelocity.minus(flippedVelocity);
        if (desiredAccel.getNorm() > maxAcceleration * 0.04)
            return flippedVelocity.plus(AutoAlign.normalize(desiredAccel).times(maxAcceleration * 0.04));
        else return wantedVelocity;
    }
}
