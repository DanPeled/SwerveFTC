package com.danpeled.swerveftclib.Swerve.modules;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.danpeled.swerveftclib.Swerve.SwerveDriveCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Abstract class representing a swerve module in an FTC robot.
 * A swerve module consists of a drive motor for translation and a servo motor for rotation.
 * This class provides methods for controlling and getting the state of the module.
 */
public abstract class SwerveModule {

    /**
     * Number of encoder ticks per revolution of the drive motor.
     */
    protected final double TICKS_PER_REVOLUTION;

    /**
     * Wheel circumference in meters.
     */
    protected final double WHEEL_CIRCUMFERENCE;
    protected PIDFCoefficients pidfCoefficients;

    /**
     * Constructs a new SwerveModule instance.
     *
     * @param driveMotorName The name of the drive motor in the hardware map.
     * @param angleServoName The name of the angle servo in the hardware map.
     * @param hw             The hardware map used to access the robot's hardware.
     * @param coefficients   The swerve drive coefficients.
     */
    public SwerveModule(String driveMotorName, String angleServoName, HardwareMap hw, SwerveDriveCoefficients coefficients) {
        this.TICKS_PER_REVOLUTION = coefficients.TICKS_PER_REVOLUTION;
        this.WHEEL_CIRCUMFERENCE = coefficients.WHEEL_CIRCUMFERENCE;
    }

    /**
     * Sets the power and angle for the swerve module.
     *
     * @param drive The speed of the drive motor, in the range -1.0 to 1.0.
     * @param angle The angle of the servo in radians.
     */
    public abstract void setPower(double drive, double angle);

    /**
     * Gets the current state of the swerve module, including the speed and angle.
     *
     * @return The current {@link SwerveModuleState}, containing the speed in meters per second
     * and the angle as a {@link Rotation2d}.
     */
    public SwerveModuleState getState() {
        double speedMetersPerSecond = getWheelSpeed();
        double angleRadians = getWheelAngleRad();
        return new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(angleRadians * 180 / Math.PI));
    }

    /**
     * Gets the current wheel speed in meters per second.
     *
     * @return The current wheel speed in meters per second.
     */
    public abstract double getWheelSpeed();

    /**
     * Gets the current wheel angle in radians.
     *
     * @return The current wheel angle in radians.
     */
    public double getWheelAngleRad() {
        return getWheelAngleDeg() * Math.PI / 180;
    }

    /**
     * Gets the current wheel angle in degrees.
     *
     * @return The current wheel angle in degrees.
     */
    public abstract double getWheelAngleDeg();
}
