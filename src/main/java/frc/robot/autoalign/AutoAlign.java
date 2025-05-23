package frc.robot.autoalign;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldPOIs;

public class AutoAlign {
    private static final double DRIVE_MAX_VELOCITY = 3.5; // Meters/Sec
    private static final double DRIVE_MAX_ACCELERATION = 10.0; // Meters/Sec^2

    private static final double JOYSTICK_ADDITION_SCALAR = 0.8; // 2.5;

    private static final double ELEVATOR_RAISE_DISTANCE_METERS = 1.0; // For targeting
    private static final double ACCEL_LIMIT_DISTANCE_METERS = 1.5;

    /**
     * Target types for auto align
     */
    public enum Target {
        LEFT_BRANCH,
        RIGHT_BRANCH,
        ALGAE_ALIGN,
        ALGAE_GRAB,
        PROCESSOR,
        NET(true);

        private boolean moveY = false;
        public boolean allowYMovement() { return moveY; }

        Target() {}
        Target(boolean moveY) { this.moveY = moveY; }
    }

    /**
     * A key-value map storing the pose data for a given auto align target type
     */
    private static final Map<Target, List<Pose2d>> targetToPoseData = Map.of(
        // Left reef poles
        Target.LEFT_BRANCH, FieldPOIs.REEF_BRANCHES_LEFT,

        // Right reef poles
        Target.RIGHT_BRANCH, FieldPOIs.REEF_BRANCHES_RIGHT,

        // Locations for lining up to algae
        Target.ALGAE_ALIGN, FieldPOIs.ALGAE_ALIGN_LOCATIONS,

        // Locations for grabbing algae
        Target.ALGAE_GRAB, FieldPOIs.ALGAE_GRAB_LOCATIONS,

        // X alignment for net scoring
        Target.NET, List.of(FieldPOIs.NET, FieldPOIs.NET_OPPOSITE),

        // Location for processor align (flipped by field half)
        Target.PROCESSOR, List.of(FieldPOIs.PROCESSOR)
    );

    private static Pose2d targetPose; // The currently targeted position (can be null)
    private static AutoAlign.Target targetType = Target.LEFT_BRANCH; // The most recently targeted target type
    private static double scheduledElevatorHeight = 0.0;

    public static Pose2d getTargetPose() {
        return targetPose;
    }

    public static AutoAlign.Target getTargetType() {
        return targetType;
    }

    /**
     * Retrieves the nearest integer pose ID for a given auto align target type
     */
    public static int mapToPoseId(Target targetType, Drive drive, Translation2d motionVector) {
        List<Pose2d> poseData = targetToPoseData.get(targetType);
        return getNearestPose(drive.getPose(), motionVector, poseData, targetType);
    }

    /**
     * Retrieves the nearest pose for a given auto align target type
     */
    public static Pose2d mapToPose(Target targetType, Drive drive, Translation2d motionVector) {
        List<Pose2d> poseData = targetToPoseData.get(targetType);
        return poseData.get(getNearestPose(drive.getPose(), motionVector, poseData, targetType));
    }

    /**
     * Grabs a Pose2d from a pose ID and target type
     */
    public static Pose2d fromPoseId(int id, Target targetType) {
        List<Pose2d> poseData = targetToPoseData.get(targetType);
        return poseData.get(id);
    }

    public static List<Pose2d> getPosesFor(Target targetType) {
        return targetToPoseData.get(targetType);
    }

    public static Command autoAlignTo(Target targetType, RobotContainer robot) {
        return autoAlignTo(targetType, robot, () -> new Translation2d());
    }

    /**
     * Returns a command for auto aligning to the nearest avaliable pose
     * @param targetType The pose group to target
     * @param robot The robot container instance
     * @param motionSupplier A supplier for the joystick vector
     * @return The command
     */
    public static Command autoAlignTo(Target targetType, RobotContainer robot, Supplier<Translation2d> motionSupplier) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                Logger.recordOutput("AutoAlign/TargetType", targetType);
                targetPose = getFlippedPose(robot.drive, targetType, motionSupplier);
                AutoAlign.targetType = targetType;
            }),
            driveToTargetWithElevatorCommand(robot)
        );
    }

    public static Command driveToTargetCommand(RobotContainer robot) {
        // Create PID controller
        ProfiledPIDController angleController = DriveCommands.getAngleController();
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return driveToTargetCommand(() -> targetType, robot.drive, angleController);
    }

    public static Command driveToTargetWithElevatorCommand(RobotContainer robot) {
        // Construct command
        return driveToTargetCommand(robot).alongWith(Commands.run(() -> {
            Translation2d robotTranslation = robot.drive.getPose().getTranslation();
            Translation2d adjustedTranslation = new Translation2d(robotTranslation.getX(), targetType.allowYMovement() ? (ChoreoAllianceFlipUtil.shouldFlip() ? ChoreoAllianceFlipUtil.flipY(0.0) : 0.0) : robotTranslation.getY());
            if (targetPose != null && targetPose.getTranslation().getDistance(adjustedTranslation) < ELEVATOR_RAISE_DISTANCE_METERS * (targetType == Target.NET ? 0.3 : 1.0)) {
                if (targetType.name().contains("BRANCH")) setScheduledElevatorHeight(robot.elevator.getScheduledPosition());
                robot.elevator.setPosition(AutoAlign.scheduledElevatorHeight);
            }
        }));
    }

    /**
     * Grabs a new pose to target
     */
    public static Pose2d getFlippedPose(Drive drive, Target targetType, Supplier<Translation2d> motionSupplier) {
        return flipIfRed(fromPoseId(getNewTargetPoseId(drive, targetType, motionSupplier), targetType));
    }

    /**
     * Performs transformations to get the actual aligning vectors
     */
    public static int getNewTargetPoseId(Drive drive, Target targetType, Supplier<Translation2d> motionSupplier) {
        Pose2d pose = drive.getPose();
        Translation2d motionVector = new Translation2d();

        // Ignore transformations if no joystick values are inputted
        if (motionSupplier.get().getNorm() > 0.1) {
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

            // Convert joystick inputs to robot relative (otherwise robot rotation messes with targeting)
            motionVector = normalize(
                motionSupplier.get().rotateBy((isFlipped
                    ? pose.getRotation().plus(new Rotation2d(Math.PI))
                    : pose.getRotation()).unaryMinus())
            );
        }

        // Find the correct pole to target
        return mapToPoseId(targetType, drive, motionVector);
    }

    /**
     * Creates a command for driving to the currently targeted pose
     * @param drive The drive subsystem
     * @param angleController A ProfiledPIDController from DriveCommands
     * @return The command
     */
    private static Command driveToTargetCommand(Supplier<Target> targetTypeSupplier, Drive drive, ProfiledPIDController angleController) {
        // Run the command
        return Commands.runEnd(() -> {
            // Get scheduled or real elevator height depending on distance from reef
            Target targetType = targetTypeSupplier.get();
            Translation2d adjustedTranslation = new Translation2d(drive.getPose().getX(), targetType.allowYMovement() ? 0.0 : drive.getPose().getY());
            double velocityDecrease = RobotContainer.getInstance().elevator.getExtensionMeters();
            double accelDecrease = velocityDecrease;
            if (targetPose.getTranslation().getDistance(adjustedTranslation) < ACCEL_LIMIT_DISTANCE_METERS)
                accelDecrease = RobotContainer.getInstance().elevator.getScheduledPosition() * 0.6;

            // Dynamically calculate drive constraints based on elevator height
            double accelScaling = MathUtil.clamp(1 - (accelDecrease / 3.0), 0.5, 1);
            double velocityScaling = MathUtil.clamp(1 - (velocityDecrease / 2.5), 0.2, 1);

            // Update profile constraints based on calculated scalars
            TrapezoidProfile velocityProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    DRIVE_MAX_VELOCITY * velocityScaling,
                    DRIVE_MAX_ACCELERATION * accelScaling
                )
            );

            // Calculate vector to target
            Translation2d toTarget = new Translation2d(
                drive.getPose().getX() - targetPose.getX(),
                targetType.allowYMovement() ? 0 : drive.getPose().getY() - targetPose.getY()
            );

            // Calculate the robot's current speed towards the target
            double velocityTowardsGoal = drive.getLinearSpeedMetersPerSec() * dot(
                normalize(drive.getLinearSpeedsVector()),
                normalize(toTarget)
            );

            // Grab the current drive state
            TrapezoidProfile.State state = velocityProfile.calculate(0.11,
                new TrapezoidProfile.State(toTarget.getNorm(), velocityTowardsGoal),
                new TrapezoidProfile.State()
            );

            // Create a velocity vector based on the drive state's velocity
            Translation2d normalized = new Translation2d(state.velocity, toTarget.getAngle());
            if (targetType.allowYMovement()) normalized = new Translation2d(normalized.getX(),
                MathUtil.applyDeadband(RobotContainer.getInstance().joystickMotionSupplier().get().getY(), DriveCommands.DEADBAND) * (DRIVE_MAX_VELOCITY * velocityScaling)
                    * (ChoreoAllianceFlipUtil.shouldFlip() ? 1 : -1));

            // Debug info
            SmartDashboard.putNumber("AA-Position", state.position);
            SmartDashboard.putNumber("AA-Velocity", state.velocity);

            // Calculate angular speed
            double omega = angleController.calculate(drive.getRotation().getRadians(), targetPose.getRotation().getRadians());
            if (Math.abs(angleController.getPositionError()) < DriveCommands.ANGLE_DEADBAND) omega = 0.0;

            SmartDashboard.putNumber("AA-CurrentAngle", drive.getRotation().getRadians());
            SmartDashboard.putNumber("AA-TargetAngle", targetPose.getRotation().getRadians());

            // Check if it's red alliance
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

            // Limit acceleration
            Translation2d finalVelocity = DriveCommands.limitAccelerationFor(
                ChoreoAllianceFlipUtil.shouldFlip() ? new Translation2d().minus(drive.getLinearSpeedsVector()) : drive.getLinearSpeedsVector(),
                normalized, DRIVE_MAX_ACCELERATION
            );

            // Convert to field relative speeds
            ChassisSpeeds speeds = new ChassisSpeeds(
                finalVelocity.getX() * (isFlipped ? -1 : 1),
                finalVelocity.getY() * (isFlipped ? -1 : 1),
                omega
            );

            // Drive the robot to targets
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()
            ));

        }, () -> targetPose = null, drive)
        .beforeStarting(() -> {
            // Reset pid controllers
            angleController.reset(
                drive.getRotation().getRadians(),
                drive.getAngularSpeedRadsPerSec()
            );
        });
    }

    /**
     * Loops through the poses and weighs them all, returning the ID of the highest weighted pose
     */
    public static int getNearestPose(Pose2d pose, Translation2d motionVector, List<Pose2d> locations, Target targetType) {
        Pair<Integer, Double> bestResult = new Pair<>(0, 1000.0);

        for (int i = 0; i < locations.size(); i++) {
            // Calculate the distance from the robot to the current reef pole
            Pose2d targetPose = AutoAlign.flipIfRed(locations.get(i));
            if (targetType.allowYMovement()) targetPose = new Pose2d(targetPose.getX(), pose.getY(), targetPose.getRotation());
            double dst = pose.getTranslation().getDistance(targetPose.getTranslation());

            if (targetType.name().contains("BRANCH")) {
                int targetFace = getReefFace(targetPose);
                if (targetFace != getReefFace(pose)) continue;
            }

            // Calculate the additional weighting based on joystick angle
            double addition = AutoAlign.calculateBestPoseAddition(
                targetPose.minus(pose).getTranslation(), motionVector);

            // Apply addition and assign new best result if applicable
            double weight = dst + addition * JOYSTICK_ADDITION_SCALAR;
            if (weight <= bestResult.getSecond()) bestResult = new Pair<>(i, weight);
        }

        return bestResult.getFirst();
    }

    public static int getReefFace(Pose2d pose) {
        Translation2d center = flipIfRed(FieldPOIs.REEF_CENTER).getTranslation();
        Translation2d robot = pose.getTranslation();

        // Calculate the angle between the reef and the robot
        Rotation2d angle = Rotation2d.fromRadians(Math.atan2(center.getY() - robot.getY(), center.getX() - robot.getX()));
        double rads = angle.getRadians() + Math.PI + (Math.PI / 6);

        // Divide the angle by π/3 to find the reef face
        int face = ((int) (rads / (Math.PI / 3))) % 6;
        Logger.recordOutput("AutoAlign/ReefFace", face);
        return face;
    }

    /**
     * Calculates a value to add to the selection "weight" of each reef pole.
     * This is determined by the dot product of the robot to reef pole vector and the vector of the
     * joystick motion. This is to ensure that the robot will prefer to target reef faces that the
     * driver is moving towards.
     */
    public static double calculateBestPoseAddition(Translation2d toReefVector, Translation2d motionVector) {
        if (motionVector.getNorm() < 0.1) return 0.0;
        return dot(normalize(toReefVector), normalize(motionVector));
    }

    /**
     * @return A copy of the given translation vector with a magnitude of 1
     */
    public static Translation2d normalize(Translation2d vector) {
        return vector.div(vector.getNorm());
    }

    /**
     * @return The dot product of translations a and b
     */
    public static double dot(Translation2d a, Translation2d b) {
        return (a.getX() * b.getX() + a.getY() * b.getY());
    }

    /**
     * Uses choreo utility methods to flip the given pose if on red alliance
     */
    public static Pose2d flipIfRed(Pose2d pose) {
        return ChoreoAllianceFlipUtil.shouldFlip() ? ChoreoAllianceFlipUtil.flip(pose) : pose;
    }

    public static void setScheduledElevatorHeight(double elevatorHeight) {
        AutoAlign.scheduledElevatorHeight = elevatorHeight;
    }

    public static Command clearTargetType() {
        return Commands.runOnce(() -> AutoAlign.targetType = Target.LEFT_BRANCH);
    }

    public static Command setTargetType(Target target) {
        return Commands.runOnce(() -> AutoAlign.targetType = target);
    }

    public static void setTargetPose(Pose2d pose) {
        AutoAlign.targetPose = pose;
    }
}
