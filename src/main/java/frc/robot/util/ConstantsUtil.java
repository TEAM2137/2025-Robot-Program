package frc.robot.util;

import frc.robot.Constants;

public class ConstantsUtil {
    /** Gets a constant based on the robot implementation (real, sim) */
    public static <T> T getRealOrSimConstant(T real, T sim) {
        return getConstant(real, real, sim);
    }

    /** Gets a hardware-only constant based on the current robot type (comp, practice) */
    public static <T> T getConstant(T comp, T practice) {
        return getConstant(comp, practice, comp);
    }

    /** Gets a constant based on the current robot type (comp, practice, sim)*/
    public static <T> T getConstant(T comp, T practice, T sim) {
        return switch (Constants.currentRobotType) {
            case COMP -> comp;
            case PRACTICE -> practice;
            case SIM -> sim;
        };
    }
}
