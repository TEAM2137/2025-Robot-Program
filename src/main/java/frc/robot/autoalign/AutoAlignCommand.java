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
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class AutoAlignCommand extends Command {
    private final Pose2d finalPose;
    private final Translation2d finalVelocity;
    private final double speedLimit;
    private final double accelerationLimit;
    private final double tolerance;
    private final double timeout;

    private final List<CommandMarker> markers;

    private final Drive drive;

    private AutoAlignCommand(Builder builder) {
        this.finalPose = builder.finalPose;
        this.finalVelocity = builder.finalVelocity;
        this.speedLimit = builder.speedLimit;
        this.accelerationLimit = builder.accelerationLimit;
        this.tolerance = builder.tolerance;
        this.timeout = builder.timeout;
        this.markers = builder.markers;

        this.drive = RobotContainer.getInstance().drive;
    }

    private Pose2d startPose;
    private Translation2d pathDir;
    private TrapezoidProfile.State pState;
    private double pathLength;

    private Timer timer;

    @Override
    public void initialize() {
        startPose = drive.getPose();

        Translation2d delta = finalPose.getTranslation().minus(startPose.getTranslation());
        pathLength = delta.getNorm(); // in meters
        pathDir = delta.div(pathLength); // normalized direction

        Logger.recordOutput("AutoAlign/startPose", startPose);
        Logger.recordOutput("AutoAlign/pathLength", pathLength);

        pState = new TrapezoidProfile.State(0.0, 0.0); // p = 0, dp = 0

        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        // loop cycle is 0.02, but use a little extra to "look ahead" in the profile
        double dt = 0.04;

        // calculate current progress
        Translation2d robotPos = drive.getPose().getTranslation();
        double alongPath = dot(robotPos.minus(startPose.getTranslation()), pathDir);
        double pCurrent = MathUtil.clamp(alongPath / pathLength, 0.0, 1.0);

        Translation2d robotVel = drive.getLinearSpeedsVector();
        double dpCurrent = dot(robotVel, pathDir) / pathLength;

        Logger.recordOutput("AutoAlign/alongPath", alongPath);

        // rebuild trapezoid each cycle to allow for error correction
        TrapezoidProfile sProfile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                        speedLimit / pathLength,
                        accelerationLimit / pathLength
                )
        );

        // advance 1 step from current state
        pState = sProfile.calculate(dt, // how far to advance in the profile
                new TrapezoidProfile.State(pCurrent, dpCurrent), // current (start)
                new TrapezoidProfile.State(1.0, 0.0) // goal
        );

        double p = pState.position;
        double dp = pState.velocity;

        Logger.recordOutput("AutoAlign/p", p);
        Logger.recordOutput("AutoAlign/dp", dp);

        // get desired position (unused rn)
//        Translation2d startPos = startPose.getTranslation();
//        Translation2d endPos = finalPose.getTranslation();
//        Translation2d desiredPos = startPos.plus(endPos.minus(startPos).times(p));

        // get desired velocity
        double commandedSpeed = dp * pathLength;
        Translation2d commandedVel = pathDir.times(commandedSpeed);

        // set velocity of the drivetrain
        drive.runVelocity(new ChassisSpeeds(
                commandedVel.getX(),
                commandedVel.getY(),
                0.0
        ));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeout);
    }

    public record CommandMarker(TriggerType type, double value, Command command) {
        public enum TriggerType { PROGRESS, TIME_AFTER_START, TIME_BEFORE_END }
    }

    public static class Builder {
        private Pose2d finalPose = new Pose2d();
        private Translation2d finalVelocity = new Translation2d();
        private double speedLimit = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // m/s
        private double accelerationLimit = 7.5; // m/s^2
        private double tolerance = 0; // meters/radians
        private double timeout = Double.POSITIVE_INFINITY; // seconds

        private final List<CommandMarker> markers = new ArrayList<>();

        public Builder withFinalPose(Pose2d pose) {
            this.finalPose = pose;
            return this;
        }

        public Builder withFinalVelocity(Translation2d velocity) {
            this.finalVelocity = velocity;
            return this;
        }

        public Builder withSpeedLimit(double speed) {
            this.speedLimit = speed;
            return this;
        }

        public Builder withAccelerationLimit(double accel) {
            this.accelerationLimit = accel;
            return this;
        }

        public Builder withEndTolerance(double tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        public Builder withTimeout(double seconds) {
            this.timeout = seconds;
            return this;
        }

        public Builder runCommandAt(double progress, Command command) {
            markers.add(new CommandMarker(CommandMarker.TriggerType.PROGRESS, progress, command));
            return this;
        }

        public Builder runCommandAtTime(double seconds, Command command) {
            markers.add(new CommandMarker(CommandMarker.TriggerType.TIME_AFTER_START, seconds, command));
            return this;
        }

        public Builder runCommandAtTimeBeforeEnd(double seconds, Command command) {
            markers.add(new CommandMarker(CommandMarker.TriggerType.TIME_BEFORE_END, seconds, command));
            return this;
        }

        public AutoAlignCommand build() {
            return new AutoAlignCommand(this);
        }
    }

    public static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }
}