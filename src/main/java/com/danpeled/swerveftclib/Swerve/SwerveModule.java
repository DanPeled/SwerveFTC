package com.danpeled.swerveftclib.Swerve;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SwerveModule {
    // Constants for motor and servo control
    private static final double TICKS_PER_REVOLUTION = 537.6; // For a typical motor with an encoder
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * 0.1; // Wheel circumference in meters (example)
    private final ServoEx m_angleServo; // The rotation axis servo
    private final DcMotorEx m_driveMotor; // Drive motor

    public SwerveModule(String driveMotorName, String angleServoName, HardwareMap hw) {
        m_driveMotor = hw.get(DcMotorEx.class, driveMotorName);
        m_angleServo = hw.get(ServoEx.class, angleServoName);

        // Configure the drive motor
        m_driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_driveMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_driveMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1, 0, 0, 0));
    }

    /**
     * @param drive speed of the drive motor (range: -1.0 to 1.0)
     * @param angle Angle of the servo in radians
     */
    public void setPower(double drive, double angle) {
        m_angleServo.turnToAngle(angle * 180 / Math.PI);
        m_driveMotor.setPower(drive);
    }

    /**
     * Gets the current state of the swerve module.
     *
     * @return The SwerveModuleState containing speed in meters per second and angle in Rotation2d
     */
    public SwerveModuleState getState() {
        double speedMetersPerSecond = getWheelSpeed();
        double angleRadians = getWheelAngleRad();
        return new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(angleRadians * 180 / Math.PI));
    }

    /**
     * Gets the current wheel speed in meters per second.
     *
     * @return The wheel speed in meters per second
     */
    private double getWheelSpeed() {
        double motorTicksPerSecond = m_driveMotor.getVelocity();
        double wheelRevolutionsPerSecond = motorTicksPerSecond / TICKS_PER_REVOLUTION;
        return wheelRevolutionsPerSecond * WHEEL_CIRCUMFERENCE;
    }

    /**
     * Gets the current wheel angle in radians.
     *
     * @return The wheel angle in radians
     */
    public double getWheelAngleRad() {
        return getWheelAngleDeg() * Math.PI / 180;
    }

    /**
     * Gets the current wheel angle in radians.
     *
     * @return The wheel angle in degrees
     */
    public double getWheelAngleDeg() {
        return m_angleServo.getAngle();
    }
}
