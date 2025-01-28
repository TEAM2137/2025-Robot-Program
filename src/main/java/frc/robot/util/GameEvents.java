package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GameEvents {
    private static boolean isAutonomous;
    private static boolean isTeleop;

    public static Trigger teleop() {
        return new Trigger(() -> isTeleop);
    }

    public static Trigger autonomous() {
        return new Trigger(() -> isAutonomous);
    }

    public static void setIsAutonomous(boolean value) {
        GameEvents.isAutonomous = value;
    }

    public static void setIsTeleop(boolean value) {
        GameEvents.isTeleop = value;
    }
}
