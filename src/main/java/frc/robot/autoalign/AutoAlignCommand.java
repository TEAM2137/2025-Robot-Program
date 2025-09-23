package frc.robot.autoalign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ConstantsUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * A versatile command class for automatically driving the robot to a point
 * on the field. Use {@code AutoAlignCommand.builder()} to create an instance.
 *
 * @author Avery Gardner
 * @since 2025
 */
public class AutoAlignCommand extends Command {
    private final TargetSelector targetSelector;
    private final Translation2d finalVelocity;
    private final double speedLimit;
    private final double accelerationLimit;
    private final double endTolerance;
    private final double timeout;
    private final List<CommandMarker> markers;
    private final Drive drive;

    private ArrayList<CommandMarker> pendingCommandMarkers;

    private Pose2d targetPose;
    private Pose2d startPose;

    private double pathLength;
    private double error;
    private double progress;

    private Timer timer;
    private double totalPathTime;

    /** Creates a new auto align command builder */
    public static Builder builder() {
        return new AutoAlignCommand.Builder();
    }

    private AutoAlignCommand(Builder builder) {
        this.targetSelector = builder.targetSelector;
        this.targetPose = builder.targetPose;
        this.finalVelocity = builder.finalVelocity;
        this.speedLimit = builder.speedLimit;
        this.accelerationLimit = builder.accelerationLimit;
        this.endTolerance = builder.endTolerance;
        this.timeout = builder.timeout;
        this.markers = builder.markers;

        this.drive = RobotContainer.getInstance().drive;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        this.startPose = drive.getPose();
        if (targetSelector != null) targetPose = targetSelector.getPose(drive.getTargetingContext());

        pathLength = getPathLength(targetPose);
        totalPathTime = 0; // gets updated in execute()

        Logger.recordOutput("AutoAlign/startPose", startPose);
        Logger.recordOutput("AutoAlign/pathLength", pathLength);

        pendingCommandMarkers = new ArrayList<>(markers);

        timer = new Timer();
        timer.start();

        Logger.recordOutput("AutoAlign/running", true);
    }

    @Override
    public void execute() {
        // "delta time" for the trapezoidal motion profiles.
        // loop cycle is 0.02s, but any extra allows you to "look ahead" in the profile.
        // more dt reduces oscillation around the target pose, but too much dt
        // messes with acceleration limits. look at log outputs and find a balance.
        // from what I've seen, you want both sides of the velocity trapezoid to have
        // similar steepness for the smoothest results.
        // simulation generally needs a lower dt than the real robot
        double dt = ConstantsUtil.getRealOrSimConstant(0.11, 0.04);

        // current pose + velocity
        Translation2d robotPos = drive.getPose().getTranslation();
        Translation2d robotVel = drive.getLinearSpeedsVector();

        // update target pose (if applicable)
        if (targetSelector != null && targetSelector.isDynamic())
            this.targetPose = targetSelector.getPose(drive.getTargetingContext());

        // calculate deltas to goal
        double dxRemaining = targetPose.getX() - robotPos.getX();
        double dyRemaining = targetPose.getY() - robotPos.getY();

        Logger.recordOutput("AutoAlign/x", dxRemaining);
        Logger.recordOutput("AutoAlign/y", dyRemaining);

        // calculate position error and progress along path
        this.pathLength = getPathLength(targetPose);
        this.error = Math.hypot(dxRemaining, dyRemaining);
        this.progress = (pathLength - error) / pathLength;

        Logger.recordOutput("AutoAlign/error", error);
        Logger.recordOutput("AutoAlign/progress", progress);

        // get current velocity
        double vxCurrent = robotVel.getX();
        double vyCurrent = robotVel.getY();

        // scale constraints so combined speed is under the limit.
        // this is necessary because x and y act independently, and ensures
        // that we don't move at sqrt(2) times the speed when moving diagonally
        double totalRemaining = Math.hypot(dxRemaining, dyRemaining);
        double scaleX = (totalRemaining > 1e-6) ? Math.abs(dxRemaining) / totalRemaining : 0.0;
        double scaleY = (totalRemaining > 1e-6) ? Math.abs(dyRemaining) / totalRemaining : 0.0;

        // create trapezoidal velocity profiles for x and y components.
        // creating a new profile each cycle ensures that we won't get off
        TrapezoidProfile xProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                speedLimit * scaleX, accelerationLimit * scaleX));
        TrapezoidProfile yProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                speedLimit * scaleY, accelerationLimit * scaleY));

        // target velocity for each axis
        double vxGoal = finalVelocity.getX();
        double vyGoal = finalVelocity.getY();

        // advance the profiles
        TrapezoidProfile.State xState = xProfile.calculate(
                dt, // delta time
                new TrapezoidProfile.State(0.0, vxCurrent), // current
                new TrapezoidProfile.State(dxRemaining, vxGoal) // goal
        );
        TrapezoidProfile.State yState = yProfile.calculate(
                dt, // delta time
                new TrapezoidProfile.State(0.0, vyCurrent), // current
                new TrapezoidProfile.State(dyRemaining, vyGoal) // goal
        );

        // apply resulting x and y velocities to drivetrain
        double vx = xState.velocity;
        double vy = yState.velocity;
        drive.runVelocity(new ChassisSpeeds(vx, vy, 0.0));

        Logger.recordOutput("AutoAlign/vx", vx);
        Logger.recordOutput("AutoAlign/vy", vy);

        // sets the path time to the total time of the longest-lasting profile
        // TODO: sometimes broken with nonzero initial velocity
        double xTime = trapezoidTimeRemaining(dxRemaining, vxCurrent, vxGoal,
                speedLimit * scaleX, accelerationLimit * scaleX);
        double yTime = trapezoidTimeRemaining(dyRemaining, vyCurrent, vyGoal,
                speedLimit * scaleY, accelerationLimit * scaleY);
        this.totalPathTime = Math.max(totalPathTime, Math.max(xTime, yTime));

        Logger.recordOutput("AutoAlign/xTime", xTime);
        Logger.recordOutput("AutoAlign/yTime", yTime);
        Logger.recordOutput("AutoAlign/totalPathTime", totalPathTime);

        // run pending commands (if applicable)
        checkPendingCommands();
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("AutoAlign/running", false);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return error < endTolerance || (timeout < 0
                ? timer.hasElapsed(totalPathTime)
                : timer.hasElapsed(timeout));
    }

    private void checkPendingCommands() {
        Iterator<CommandMarker> it = pendingCommandMarkers.iterator();
        while (it.hasNext()) {
            CommandMarker marker = it.next();
            switch (marker.type) {
                case PROGRESS -> {
                    if (progress > marker.value) {
                        marker.command.schedule();
                        it.remove();
                    }
                }
                case DISTANCE -> {
                    if (error < marker.value) {
                        marker.command.schedule();
                        it.remove();
                    }
                }
                case TIME_AFTER_START -> {
                    if (timer.hasElapsed(marker.value)) {
                        marker.command.schedule();
                        it.remove();
                    }
                }
                case TIME_BEFORE_END -> {
                    if (timer.hasElapsed(totalPathTime - marker.value)) {
                        marker.command.schedule();
                        it.remove();
                    }
                }
            }
        }
    }

    private double getPathLength(Pose2d target) {
        return target.getTranslation().minus(startPose.getTranslation()).getNorm();
    }

    // thanks chatgpt lol
    public static double trapezoidTimeRemaining(double dst, double v0, double vf,
                                                double vMax, double aMax) {
        if (Math.abs(dst) < 0.005) return 0.0;
        if (dst < 0) {
            dst = -dst;
            v0 = -v0;
            vf = -vf;
        }
        v0 = Math.max(0, v0);
        vf = Math.max(0, vf);
        double dAccel = (vMax * vMax - v0 * v0) / (2.0 * aMax);
        double dDecel = (vMax * vMax - vf * vf) / (2.0 * aMax);
        if (dst > dAccel + dDecel) {
            double tAccel = (vMax - v0) / aMax;
            double tDecel = (vMax - vf) / aMax;
            double tCruise = (dst - dAccel - dDecel) / vMax;
            return tAccel + tCruise + tDecel;
        } else {
            double vPeak = Math.sqrt(aMax * dst + 0.5 * (v0 * v0 + vf * vf));
            double tAccel = (vPeak - v0) / aMax;
            double tDecel = (vPeak - vf) / aMax;
            return tAccel + tDecel;
        }
    }

    public record CommandMarker(TriggerType type, double value, Command command) {
        public enum TriggerType { PROGRESS, DISTANCE, TIME_AFTER_START, TIME_BEFORE_END }
    }

    public static class Builder {
        // default values
        private TargetSelector targetSelector;
        private Pose2d targetPose = new Pose2d();
        private Translation2d finalVelocity = new Translation2d();
        private double speedLimit = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // m/s
        private double accelerationLimit = 7.5; // m/s^2
        private double endTolerance = -1; // meters
        private double timeout = Double.POSITIVE_INFINITY; // seconds
        private final List<CommandMarker> markers = new ArrayList<>();

        /** Use {@code AutoAlignCommand.builder()} instead */
        private Builder() {}

        /**
         * Sets the {@code TargetSelector} responsible for selecting a Pose2D to target.
         * For situations with only one possible target, use {@code builder.withTargetPose()}
         */
        public Builder withTargetSelector(TargetSelector selector) {
            this.targetSelector = selector;
            return this;
        }

        /**
         * Sets this command's target position and rotation of the robot
         */
        public Builder withTargetPose(Pose2d pose) {
            this.targetPose = pose;
            return this;
        }

        /** Decorates this command to roughly end with the given velocity
         * (meters/second) */
        public Builder withFinalVelocity(Translation2d velocity) {
            this.finalVelocity = velocity;
            return this;
        }

        /**
         * Decorates this command to not drive faster than the given limit
         * (meters/second)
         */
        public Builder withSpeedLimit(double speed) {
            this.speedLimit = speed;
            return this;
        }

        /**
         * Decorates this command to not accelerate faster than the given limit
         * (meters/second^2)
         */
        public Builder withAccelerationLimit(double accel) {
            this.accelerationLimit = accel;
            return this;
        }

        /**
         * Decorates this command to end when the robot is within a given distance
         * (meters) to the target
         */
        public Builder withEndTolerance(double tolerance) {
            this.endTolerance = tolerance;
            return this;
        }

        /** Decorates this command to end after a given amount of time has passed */
        public Builder withTimeout(double seconds) {
            this.timeout = seconds;
            return this;
        }

        /** Decorates this command to end after it's total estimated time expires */
        public Builder endWhenFinished() {
            this.timeout = -1;
            return this;
        }

        /**
         * Decorates this command to run another command once the robot has completed
         * a given percent of the path (0.0 - 1.0)
         */
        public Builder runCommandAt(double progress, Command command) {
            return runCommand(CommandMarker.TriggerType.PROGRESS, progress, command);
        }

        /**
         * Decorates this command to run another command once the robot is a given
         * distance from the target (meters)
         */
        public Builder runCommandAtDistance(double distance, Command command) {
            return runCommand(CommandMarker.TriggerType.DISTANCE, distance, command);
        }

        /**
         * Decorates this command to run another command once a given amount of
         * time has passed (seconds)
         * */
        public Builder runCommandAtTime(double seconds, Command command) {
            return runCommand(CommandMarker.TriggerType.TIME_AFTER_START, seconds, command);
        }

        /**
         * Decorates this command to run another command a given amount of time
         * before the path ends (seconds)
         */
        public Builder runCommandAtTimeBeforeEnd(double seconds, Command command) {
            return runCommand(CommandMarker.TriggerType.TIME_BEFORE_END, seconds, command);
        }

        private Builder runCommand(CommandMarker.TriggerType type, double seconds, Command command) {
            markers.add(new CommandMarker(type, seconds, command));
            return this;
        }

        /** Creates an {@code AutoAlignCommand} from this builder */
        public AutoAlignCommand build() {
            return new AutoAlignCommand(this);
        }

        /**
         * Creates an {@code AutoAlignCommand} from this builder
         * @param name the internal name to give to the newly created command
         */
        public AutoAlignCommand build(String name) {
            AutoAlignCommand command = build();
            command.setName(name);
            return command;
        }
    }
}
