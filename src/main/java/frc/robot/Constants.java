package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    // The serial number of the practice bot roboRIO
    private static final String practiceRioSerialNumber = "03415A1A";

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final RobotType currentRobotType = getRobotType();

    public static RobotType getRobotType() {
        if (currentMode != Mode.SIM) {
            LoggedNetworkString loggedSerialNumber = new LoggedNetworkString(
                    "RioSerialNumber", HALUtil.getSerialNumber());
            return loggedSerialNumber.get().matches(practiceRioSerialNumber)
                    ? RobotType.PRACTICE
                    : RobotType.COMP;
        }
        return RobotType.SIM;
    }

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public enum RobotType {
        /** Running the competition robot */
        COMP,

        /** Running the practice robot */
        PRACTICE,

        /** Running a simulated robot */
        SIM
    }
}
