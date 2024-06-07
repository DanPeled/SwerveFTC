package org.firstinspires.ftc.teamcode.util;

/**
 * Utility class for common mathematical operations.
 */
public class MathUtil {

    /**
     * Clamps a value within a specified range.
     *
     * @param value The value to clamp.
     * @param min   The minimum value of the range.
     * @param max   The maximum value of the range.
     * @return The clamped value.
     */
    public static double clamp(double value, double min, double max) {
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return Math.max(Math.min(value, max), min);
    }

    /**
     * Clamps an integer value within a specified range.
     *
     * @param value The value to clamp.
     * @param min   The minimum value of the range.
     * @param max   The maximum value of the range.
     * @return The clamped value.
     */
    public static int clamp(int value, int min, int max) {
        if (min > max) {
            int tempMin = min;
            min = max;
            max = tempMin;
        }
        return Math.max(Math.min(value, max), min);
    }

    /**
     * Checks if a value is within a specified range.
     *
     * @param value The value to check.
     * @param min   The minimum value of the range.
     * @param max   The maximum value of the range.
     * @return {@code true} if the value is within the range, otherwise {@code false}.
     */
    public static boolean inRange(double value, double min, double max) {
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return value > min && value < max;
    }

    /**
     * Checks if a value is outside of a specified range.
     *
     * @param value The value to check.
     * @param min   The minimum value of the range.
     * @param max   The maximum value of the range.
     * @return {@code true} if the value is outside the range, otherwise {@code false}.
     */
    public static boolean outOfRange(double value, double min, double max) {
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return value <= min || value >= max;
    }

    /**
     * Checks if a value is approximately equal to a target value within a specified approximation.
     *
     * @param value         The value to check.
     * @param targetValue   The target value.
     * @param approximation The allowed approximation.
     * @return {@code true} if the value is approximately equal to the target value, otherwise {@code false}.
     */
    public static boolean approximately(double value, double targetValue, double approximation) {
        return value > (targetValue - approximation) && value < (targetValue + approximation);
    }

    /**
     * Maps a value from one range to another range.
     *
     * @param x    The value to map.
     * @param minX The minimum value of the original range.
     * @param maxX The maximum value of the original range.
     * @param minY The minimum value of the target range.
     * @param maxY The maximum value of the target range.
     * @return The mapped value.
     */
    public static double map(double x, double minX, double maxX, double minY, double maxY) {
        if (maxX == minX) {
            return 0;
        }
        // Scale the value to a 0-1 range based on its position in the original range
        double normalizedValue = (x - minX) / (maxX - minX);

        // Map the normalized value to the new range
        return normalizedValue * (maxY - minY) + minY;
    }
}
