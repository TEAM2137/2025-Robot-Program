package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class AutoAlignUtil {
    /**
     * Calculates a value to add to the selection "weight" of each reef pole.
     * This is determined by the dot product of the robot to reef pole vector and the vector of the
     * joystick motion. This is to ensure that the robot will prefer to target reef faces that the
     * driver is moving towards.
     */
    public static double calculateBestReefPoleAddition(Translation2d toReefVector, Translation2d motionVector) {
        if (motionVector.getNorm() < 0.1) return 0.0;
        return dot(normalize(toReefVector), normalize(motionVector));
    }

    public static Translation2d normalize(Translation2d vector) {
        return vector.div(vector.getNorm());
    }

    public static double dot(Translation2d a, Translation2d b) {
        return (a.getX() * b.getX() + a.getY() * b.getY());
    }
}
