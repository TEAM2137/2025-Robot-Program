package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AutoAlignUtil;
import frc.robot.util.FieldPOIs;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY = new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS =
        Math.max(
            Math.max(
                Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY))
        );

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();

    // Auto trajectory following PIDs
    private final PIDController autoXController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController autoYController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController autoHeadingController = new PIDController(7.5, 0.0, 0.0);

    private final Field2d field = new Field2d();
    private final Sendable swerveDriveSendable = new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> modules[0].getAngle().getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> modules[0].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty("Front Right Angle", () -> modules[1].getAngle().getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> modules[1].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty("Back Left Angle", () -> modules[2].getAngle().getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> modules[2].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty("Back Right Angle", () -> modules[3].getAngle().getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> modules[3].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
        }
    };

    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    private static StructPublisher<Pose2d> closestLeftPolePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("ClosestLeftPole", Pose2d.struct).publish();
    private static StructPublisher<Pose2d> closestRightPolePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("ClosestRightPole", Pose2d.struct).publish();
    private static StructArrayPublisher<Translation2d> joystickPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("JoystickMotionVector", Translation2d.struct).publish();
    private static StructArrayPublisher<Translation2d> toLeftReefPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("ToLeftReefVector", Translation2d.struct).publish();
    private static StructArrayPublisher<Translation2d> toRightReefPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("ToRightReefVector", Translation2d.struct).publish();

    public Drive(GyroIO gyroIO, ModuleIO flModuleIO,ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // Configure SysId
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) module.periodic();

        odometryLock.unlock();

        if (DriverStation.isDisabled()) {
            // Stop moving when disabled
            for (var module : modules) module.stop();

            // Log empty setpoint states when disabled
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;

        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(modulePositions[moduleIndex].distanceMeters
                        - lastModulePositions[moduleIndex].distanceMeters,modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

        // Post auto align debug displays in NetworkTables
        Pose2d leftPole = DriveCommands.getNewTargetPole(this, false, RobotContainer.getInstance().joystickMotionSupplier());
        Pose2d rightPole = DriveCommands.getNewTargetPole(this, true, RobotContainer.getInstance().joystickMotionSupplier());
        Pose2d offscreenPole = new Pose2d(new Translation2d(100, 100), new Rotation2d());

        boolean isTargetingLeft = RobotContainer.getInstance().targetLeft.getAsBoolean() && DriveCommands.getTarget() != null;
        boolean isTargetingRight = RobotContainer.getInstance().targetRight.getAsBoolean() && DriveCommands.getTarget() != null;

        closestLeftPolePublisher.accept(isTargetingLeft ? DriveCommands.getTarget() : offscreenPole);
        closestRightPolePublisher.accept(isTargetingRight ? DriveCommands.getTarget() : offscreenPole);

        toLeftReefPublisher.accept(createTrajectoryTo(leftPole.getTranslation()));
        toRightReefPublisher.accept(createTrajectoryTo(rightPole.getTranslation()));

        joystickPublisher.accept(createTrajectoryTo(getPose().getTranslation()
            .minus(RobotContainer.getInstance().joystickMotionSupplier().get().times(1.5))));

        // Post dashboard data through SmartDashboard
        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Swerve Drive", swerveDriveSendable);
    }

    public Translation2d[] createTrajectoryTo(Translation2d point) {
        Translation2d[] trajectory = new Translation2d[2];
        trajectory[0] = getPose().getTranslation();
        trajectory[1] = point;
        return trajectory;
    }

    public int getNearestLeftPole(Pose2d pose, Translation2d motionVector) {
        return getNearestPole(pose, motionVector, FieldPOIs.REEF_LOCATIONS_LEFT);
    }

    public int getNearestRightPole(Pose2d pose, Translation2d motionVector) {
        return getNearestPole(pose, motionVector, FieldPOIs.REEF_LOCATIONS_RIGHT);
    }

    public int getNearestPole(Pose2d pose, Translation2d motionVector, List<Pose2d> locations) {
        Pair<Integer, Double> bestResult = new Pair<>(-1, 1000.0);

        for (int i = 0; i < locations.size(); i++) {
            // Calculate the distance from the robot to the current reef pole
            Pose2d poleLocation = AutoAlignUtil.flipIfRed(locations.get(i));
            double dst = pose.getTranslation().getDistance(poleLocation.getTranslation());

            // Calculate the additional weighting based on joystick angle
            double addition = AutoAlignUtil.calculateBestReefPoleAddition(
                poleLocation.minus(pose).getTranslation(), motionVector);

            // Apply addition and assign new best result if applicable
            double weight = dst + addition * 2.8;
            if (weight <= bestResult.getSecond()) bestResult = new Pair<>(i, weight);
        }

        return bestResult.getFirst();
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public Command stopCommand() {
        return runOnce(() -> stop());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void xLock() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns a field-space vector representing the robot's linear speed in meters per sec. */
    public Translation2d getLinearSpeedsVector() {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates()), getRotation());
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    /** Returns the current linear speed in meters per sec. */
    public double getLinearSpeedMetersPerSec() {
        return getLinearSpeedsVector().getNorm();
    }

    /** Returns the current angular speed in radians per sec. */
    public double getAngularSpeedRadsPerSec() {
        return kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + autoXController.calculate(pose.getX(), sample.x),
            sample.vy + autoYController.calculate(pose.getY(), sample.y),
            sample.omega + autoHeadingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated speeds
        runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation()));
    }
}
