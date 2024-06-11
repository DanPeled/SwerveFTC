package org.firstinspires.ftc.teamcode.util.ECSSystem;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Location;

/**
 * Abstract base class for different types of drive systems in an FTC robot.
 * It provides common methods and properties that can be used by any drive system.
 */
public abstract class BaseDrive extends SubsystemBase {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected CommandOpMode robot;

    /**
     * Sets the power for the drive system.
     *
     * @param x    The power in the x direction (forward/backward).
     * @param y    The power in the y direction (left/right).
     * @param turn The power for turning (rotation).
     */
    public abstract void setPower(double x, double y, double turn);

    /**
     * Sets the power for the drive system with an option for field-oriented control.
     *
     * @param x             The power in the x direction (forward/backward).
     * @param y             The power in the y direction (left/right).
     * @param turn          The power for turning (rotation).
     * @param fieldOriented If true, the control is field-oriented; otherwise, it is robot-oriented.
     */
    public abstract void setPowerOriented(double x, double y, double turn, boolean fieldOriented);

    /**
     * Gets the current X position of the robot.
     *
     * @return The X position.
     */
    public abstract double getPosX();

    /**
     * Gets the current Y position of the robot.
     *
     * @return The Y position.
     */
    public abstract double getPosY();

    /**
     * Gets the current heading (orientation) of the robot.
     *
     * @return The heading in degrees.
     */
    public abstract double getHeading();

    /**
     * Resets the robot's orientation to a specified angle.
     *
     * @param angle The new orientation angle in degrees.
     */
    public abstract void resetOrientation(double angle);

    /**
     * Stops the robot by setting all power to zero.
     */
    public void stopPower() {
        setPower(0, 0, 0);
    }

    /**
     * Turns the robot by a specified number of degrees with a specified power.
     *
     * @param deg   The number of degrees to turn.
     * @param power The power to use for the turn.
     */
    public void turn(double deg, double power) {
        double targetAngle = getHeading() + deg; // zeroAngle
        turnTo(targetAngle, power);
    }

    /**
     * Calculates the delta heading (difference) between the current heading and a target heading.
     *
     * @param target The target heading in degrees.
     * @return The delta heading in degrees.
     */
    public double getDeltaHeading(double target) {
        double currentAngle = getHeading();
        double delta = (target - currentAngle) % 360;

        if (delta < -180) {
            delta = delta + 360;
        }
        if (delta > 180) {
            delta = delta - 360;
        }
        return delta;
    }

    /**
     * Turns the robot to a specified target angle with a specified target power.
     *
     * @param targetAngle The target angle in degrees.
     * @param targetPower The target power for the turn.
     */
    public void turnTo(double targetAngle, double targetPower) {
        double delta = getDeltaHeading(targetAngle);
        double s = (delta < 0) ? -1 : 1;
        while (robot.opModeIsActive() && (delta * s > 0)) {
            delta = getDeltaHeading(targetAngle);
            double gain = 0.02;
            double power = gain * delta * targetPower;
            if (Math.abs(power) < 0.1) power = 0.1 * Math.signum(power);

            setPower(0, power, 0);

            robot.telemetry.addData("target", targetAngle);
            robot.telemetry.addData("current", getHeading());
            robot.telemetry.addData("delta", delta);
            robot.telemetry.addData("power", power);
            robot.telemetry.update();
        }
        stopPower();
    }

    /**
     * Moves the robot to a specified location with specified settings and an optional midway action.
     *
     * @param location     The target location.
     * @param settings     The settings for the movement.
     * @param midwayAction An optional action to perform midway through the movement.
     */
    public abstract void goToLocation(Location location, GotoSettings settings, Runnable midwayAction);

    /**
     * Moves the robot to a specified location with specified settings.
     *
     * @param location The target location.
     * @param settings The settings for the movement.
     */
    public abstract void goToLocation(Location location, GotoSettings settings);

    /**
     * Settings for the goToLocation method, including power, tolerance, timeout, and slowdown mode.
     */
    public static class GotoSettings {
        public double power, tolerance, timeout;
        public boolean noSlowdown;

        /**
         * Constructor for GotoSettings.
         *
         * @param power      The power for the movement.
         * @param tolerance  The tolerance for the target location.
         * @param timeout    The timeout for the movement.
         * @param noSlowdown If true, the robot will not slow down when approaching the target.
         */
        public GotoSettings(double power, double tolerance, double timeout, boolean noSlowdown) {
            this.power = power;
            this.tolerance = tolerance;
            this.timeout = timeout;
            this.noSlowdown = noSlowdown;
        }

        /**
         * Builder class for creating GotoSettings instances.
         */
        public static class Builder {
            double power, tolerance, timeout;
            boolean noSlowdown;

            /**
             * Sets the power for the movement.
             *
             * @param power The power for the movement.
             * @return The Builder instance.
             */
            public Builder setPower(double power) {
                this.power = power;
                return this;
            }

            /**
             * Sets the tolerance for the target location.
             *
             * @param tolerance The tolerance for the target location.
             * @return The Builder instance.
             */
            public Builder setTolerance(double tolerance) {
                this.tolerance = tolerance;
                return this;
            }

            /**
             * Sets the timeout for the movement.
             *
             * @param timeout The timeout for the movement.
             * @return The Builder instance.
             */
            public Builder setTimeout(double timeout) {
                this.timeout = timeout;
                return this;
            }

            /**
             * Sets whether the robot should slow down when approaching the target.
             *
             * @param active If true, the robot will not slow down when approaching the target.
             * @return The Builder instance.
             */
            public Builder setSlowdownMode(boolean active) {
                this.noSlowdown = active;
                return this;
            }

            /**
             * Builds and returns a GotoSettings instance.
             *
             * @return The created GotoSettings instance.
             */
            public GotoSettings build() {
                try {
                    return new GotoSettings(power, tolerance, timeout, noSlowdown);
                } catch (Exception e) {
                    e.printStackTrace();
                    return null;
                }
            }
        }
    }
}
