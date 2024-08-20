package com.danpeled.swerveftclib.Swerve.modules;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.danpeled.swerveftclib.Swerve.SwerveDriveCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    protected SwerveModuleConfiguration m_configuration;

    /**
     * Constructs a new SwerveModule instance.
     *
     * @param config       The module's config parameters.
     * @param hw           The hardware map used to access the robot's hardware.
     * @param coefficients The swerve drive coefficients.
     */
    public SwerveModule(SwerveModuleConfiguration config, SwerveDriveCoefficients coefficients, HardwareMap hw) {
        this.TICKS_PER_REVOLUTION = coefficients.TICKS_PER_REVOLUTION;
        this.WHEEL_CIRCUMFERENCE = coefficients.WHEEL_CIRCUMFERENCE;

        PIDFController drivePIDF = new PIDFController(
                coefficients.drivePIDFCoefficients.p,
                coefficients.drivePIDFCoefficients.i,
                coefficients.drivePIDFCoefficients.d,
                coefficients.drivePIDFCoefficients.f
        );

        PIDFController anglePIDF = new PIDFController(
                coefficients.anglePIDCoefficients.p,
                coefficients.anglePIDCoefficients.i,
                coefficients.anglePIDCoefficients.d,
                coefficients.anglePIDCoefficients.f
        );

        m_configuration.drivePIDFController = drivePIDF;
        m_configuration.anglePIDFController = anglePIDF;
        this.m_configuration = config;
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
