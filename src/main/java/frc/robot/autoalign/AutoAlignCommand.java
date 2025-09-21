package frc.robot.autoalign;

import edu.wpi.first.math.MathUtil;
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
    private final Pose2d finalPose;
    private final Translation2d finalVelocity;
    private final double speedLimit;
    private final double accelerationLimit;
    private final double endTolerance;
    private final double timeout;

    private final List<CommandMarker> markers;

    private final Drive drive;

    /** Creates a new auto align command builder */
    public static Builder builder() {
        return new AutoAlignCommand.Builder();
    }

    private AutoAlignCommand(Builder builder) {
        this.finalPose = builder.finalPose;
        this.finalVelocity = builder.finalVelocity;
        this.speedLimit = builder.speedLimit;
        this.accelerationLimit = builder.accelerationLimit;
        this.endTolerance = builder.endTolerance;
        this.timeout = builder.timeout;
        this.markers = builder.markers;

        this.drive = RobotContainer.getInstance().drive;
        this.addRequirements(drive);
    }

    private ArrayList<CommandMarker> pendingCommandMarkers;

    private Pose2d startPose;
    private Translation2d pathDir;
    private TrapezoidProfile.State pState;

    private double pathLength;
    private double positionError;

    private Timer timer;
    private double totalPathTime;

    @Override
    public void initialize() {
        startPose = drive.getPose();

        Translation2d delta = finalPose.getTranslation().minus(startPose.getTranslation());
        pathLength = delta.getNorm(); // in meters
        pathDir = delta.div(pathLength); // normalized direction
        totalPathTime = calculateTotalPathTime();

        Logger.recordOutput("AutoAlign/startPose", startPose);
        Logger.recordOutput("AutoAlign/pathLength", pathLength);

        pState = new TrapezoidProfile.State(0.0, 0.0); // p = 0, dp = 0

        pendingCommandMarkers = new ArrayList<>(markers);

        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        // loop cycle is 0.02, but use a little extra to "look ahead" in the profile
        double dt = ConstantsUtil.getRealOrSimConstant(0.11, 0.04);

        // calculate current progress
        Translation2d robotPos = drive.getPose().getTranslation();
        double alongPath = dot(robotPos.minus(startPose.getTranslation()), pathDir);
        double pCurrent = MathUtil.clamp(alongPath / pathLength, 0.0, 1.0);

        Translation2d robotVel = drive.getLinearSpeedsVector();
        double dpCurrent = dot(robotVel, pathDir) / pathLength;
        positionError = robotPos.getDistance(finalPose.getTranslation());

        Logger.recordOutput("AutoAlign/alongPath", alongPath);
        Logger.recordOutput("AutoAlign/positionError", alongPath);

        // rebuild trapezoid each cycle to allow for error correction
        TrapezoidProfile sProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        speedLimit / pathLength,
                        accelerationLimit / pathLength
                )
        );

        // advance 1 step from current state
        double dpGoal = Math.max(0, dot(finalVelocity, pathDir) / pathLength);
        pState = sProfile.calculate(
                dt, // how far to advance in the profile
                new TrapezoidProfile.State(pCurrent, dpCurrent), // current (start)
                new TrapezoidProfile.State(1.0, dpGoal) // goal
        );

        // grab progress and delta progress from profile
        double p = pState.position;
        double dp = pState.velocity;

        Logger.recordOutput("AutoAlign/p", p);
        Logger.recordOutput("AutoAlign/dp", dp);

        // get desired velocity
        double commandedSpeed = dp * pathLength;
        Translation2d commandedVel = pathDir.times(commandedSpeed);

        // blend velocity with final velocity (bad solution, should revisit)
        Translation2d lateral = finalVelocity.minus(pathDir.times(dpGoal * pathLength));
        Translation2d blendedVel = commandedVel.plus(lateral.times(p));

        // set velocity of the drivetrain
        drive.runVelocity(new ChassisSpeeds(
                blendedVel.getX(),
                blendedVel.getY(),
                0.0
        ));

        // execute commands along path
        Iterator<CommandMarker> it = pendingCommandMarkers.iterator();
        while (it.hasNext()) {
            CommandMarker marker = it.next();
            switch (marker.type) {
                case PROGRESS -> {
                    if (p > marker.value) {
                        marker.command.schedule();
                        it.remove();
                    }
                }
                case DISTANCE -> {
                    if (positionError < marker.value) {
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

    @Override
    public boolean isFinished() {
        if (timeout < 0 && timer.hasElapsed(totalPathTime)) return true;
        return timer.hasElapsed(timeout) || positionError < endTolerance;
    }

    private double calculateTotalPathTime() {
        TrapezoidProfile profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        speedLimit / pathLength,
                        accelerationLimit / pathLength
                )
        );
        double dpGoal = Math.max(0, dot(finalVelocity, pathDir) / pathLength);
        profile.calculate(0,
                new TrapezoidProfile.State(0.0, 0.0),
                new TrapezoidProfile.State(1.0, dpGoal)
        );
        return profile.totalTime();
    }

    private static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    public record CommandMarker(TriggerType type, double value, Command command) {
        public enum TriggerType { PROGRESS, DISTANCE, TIME_AFTER_START, TIME_BEFORE_END }
    }

    public static class Builder {
        private Pose2d finalPose = new Pose2d();
        private Translation2d finalVelocity = new Translation2d();
        private double speedLimit = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // m/s
        private double accelerationLimit = 7.5; // m/s^2
        private double endTolerance = -1; // meters
        private double timeout = Double.POSITIVE_INFINITY; // seconds

        private final List<CommandMarker> markers = new ArrayList<>();

        private Builder() {}

        /**
         * Decorates this command to end at a pose
         */
        public Builder withTargetPose(Pose2d pose) {
            this.finalPose = pose;
            return this;
        }

        /** Decorates this command to roughly end with the given velocity (meters/second) */
        public Builder withFinalVelocity(Translation2d velocity) {
            this.finalVelocity = velocity;
            return this;
        }

        /** Decorates this command to not drive faster than the given limit (meters/second) */
        public Builder withSpeedLimit(double speed) {
            this.speedLimit = speed;
            return this;
        }

        /** Decorates this command to not accelerate faster than the given limit (meters/second^2) */
        public Builder withAccelerationLimit(double accel) {
            this.accelerationLimit = accel;
            return this;
        }

        /** Decorates this command to end when the robot is within a given distance (meters) to the target */
        public Builder withEndTolerance(double tolerance) {
            this.endTolerance = tolerance;
            return this;
        }

        /** Decorates this command to end after a given amount of time has passed */
        public Builder withTimeout(double seconds) {
            this.timeout = seconds;
            return this;
        }

        /**
         * Decorates this command to run another command once the robot has completed
         * a given percent of the path (0.0 - 1.0)
         */
        public Builder runCommandAt(double progress, Command command) {
            markers.add(new CommandMarker(CommandMarker.TriggerType.PROGRESS, progress, command));
            return this;
        }

        /**
         * Decorates this command to run another command once the robot is a given
         * distance from the target (meters)
         */
        public Builder runCommandAtDistance(double distance, Command command) {
            markers.add(new CommandMarker(CommandMarker.TriggerType.DISTANCE, distance, command));
            return this;
        }

        /**
         * Decorates this command to run another command once a given amount of
         * time has passed (seconds)
         * */
        public Builder runCommandAtTime(double seconds, Command command) {
            markers.add(new CommandMarker(CommandMarker.TriggerType.TIME_AFTER_START, seconds, command));
            return this;
        }

        /**
         * Decorates this command to run another command a given amount of time
         * before the path ends (seconds)
         */
        public Builder runCommandAtTimeBeforeEnd(double seconds, Command command) {
            markers.add(new CommandMarker(CommandMarker.TriggerType.TIME_BEFORE_END, seconds, command));
            return this;
        }

        /** Creates an {@code AutoAlignCommand} from this builder */
        public AutoAlignCommand build() {
            return new AutoAlignCommand(this);
        }
    }
}
