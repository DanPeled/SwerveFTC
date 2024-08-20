package com.danpeled.swerveftclib.Swerve.modules;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.danpeled.swerveftclib.Swerve.SwerveDriveCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * A SwerveModule implementation using a ServoEx for the rotation axis and a DcMotorEx for driving.
 * This class handles the control and feedback for a single swerve module.
 */
public class ServoExSwerveModule extends SwerveModule {

    /**
     * The servo responsible for rotating the module.
     */
    private final ServoEx m_angleServo;

    /**
     * The motor responsible for driving the module.
     */
    private final DcMotorEx m_driveMotor;

    /**
     * Constructs a new ServoExSwerveModule instance.
     *
     * @param driveMotorName The name of the drive motor in the hardware map.
     * @param angleServoName The name of the angle servo in the hardware map.
     * @param hw             The hardware map used to access the robot's hardware.
     * @param coefficients   The swerve drive coefficients.
     */
    public ServoExSwerveModule(String driveMotorName, String angleServoName, HardwareMap hw, SwerveDriveCoefficients coefficients) {
        super(driveMotorName, angleServoName, hw, coefficients);

        m_driveMotor = hw.get(DcMotorEx.class, driveMotorName);
        m_angleServo = hw.get(ServoEx.class, angleServoName);

        // Configure the drive motor
        m_driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_driveMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_driveMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1, 0, 0, 0));
    }

    /**
     * Sets the power and angle for the swerve module.
     *
     * @param drive The speed of the drive motor, in the range -1.0 to 1.0.
     * @param angle The angle of the servo in radians.
     */
    @Override
    public void setPower(double drive, double angle) {
        m_angleServo.turnToAngle(angle * 180 / Math.PI);
        m_driveMotor.setPower(drive);
    }

    /**
     * Gets the current state of the swerve module, including the speed and angle.
     *
     * @return The current {@link SwerveModuleState}, containing the speed in meters per second
     * and the angle as a {@link Rotation2d}.
     */
    @Override
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
    @Override
    public double getWheelSpeed() {
        double motorTicksPerSecond = m_driveMotor.getVelocity();
        double wheelRevolutionsPerSecond = motorTicksPerSecond / TICKS_PER_REVOLUTION;
        return wheelRevolutionsPerSecond * WHEEL_CIRCUMFERENCE;
    }

    /**
     * Gets the current wheel angle in radians.
     *
     * @return The current wheel angle in radians.
     */
    @Override
    public double getWheelAngleRad() {
        return getWheelAngleDeg() * Math.PI / 180;
    }

    /**
     * Gets the current wheel angle in degrees.
     *
     * @return The current wheel angle in degrees.
     */
    @Override
    public double getWheelAngleDeg() {
        return m_angleServo.getAngle();
    }
}
