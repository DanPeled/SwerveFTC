package com.danpeled.swerveftclib.Swerve;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.danpeled.swerveftclib.Swerve.modules.AxonSwerveModule;
import com.danpeled.swerveftclib.Swerve.modules.SwerveModule;
import com.danpeled.swerveftclib.Swerve.modules.SwerveModuleConfiguration;
import com.danpeled.swerveftclib.util.BaseDrive;
import com.danpeled.swerveftclib.util.Location;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * SwerveDrive class controls a swerve drive system for an FTC robot.
 *
 * @see BaseDrive
 */
public class SwerveDrive extends BaseDrive {
    /**
     * Maximum number of idle breaks before considering the robot stopped.
     */
    public final int MAX_IDLE_BREAK = 20;

    /**
     * Coefficients for the swerve drive.
     */
    public final SwerveDriveCoefficients m_coefficients;

    /**
     * Parameters for the IMU (Inertial Measurement Unit).
     */
    private final BNO055IMU.Parameters m_imuParameters = new BNO055IMU.Parameters();

    /**
     * Starting position of the robot.
     */
    private final Location m_startingPosition = new Location(0, 0);
    /**
     * Swerve Drive Odometry for the robot.
     */
    private SwerveDriveOdometry m_odometry;
    /**
     * Swerve Drive Kinematics for the robot.
     */
    private final SwerveDriveKinematics m_kinematics;
    /**
     * Instance of the IMU.
     */
    private BNO055IMU m_imu = null;
    /**
     * Swerve modules of the robot.
     */
    private SwerveModule m_fl = null, m_fr = null, m_bl = null, m_br = null;
    /**
     * Angle offset for the robot's orientation.
     */
    private double m_angleOffset = 0;
    /**
     * Current X position of the robot.
     */
    private double m_posX = 0;
    /**
     * Current Y position of the robot.
     */
    private double m_posY = 0;


    public SwerveDrive(CommandOpMode opMode, SwerveDriveCoefficients swerveDriveCoefficients) {
        this.m_coefficients = swerveDriveCoefficients;

        this.m_robot = opMode;
        this.m_hardwareMap = m_robot.hardwareMap;
        this.m_telemetry = m_robot.telemetry;

        m_kinematics = new SwerveDriveKinematics(getWheelPositions());
    }

    /**
     * Gets the wheel positions for the swerve drive system.
     *
     * @return An array of {@link Translation2d} representing the positions of the swerve drive wheels.
     */
    private Translation2d[] getWheelPositions() {
        Translation2d frontLeft = m_coefficients.FRONT_LEFT_WHEEL_POSITION;
        Translation2d frontRight = m_coefficients.FRONT_RIGHT_WHEEL_POSITION;
        Translation2d backLeft = m_coefficients.BACK_LEFT_WHEEL_POSITION;
        Translation2d backRight = m_coefficients.BACK_RIGHT_WHEEL_POSITION;

        return new Translation2d[]{frontLeft, frontRight, backLeft, backRight};
    }

    /**
     * Initializes the swerve drive modules.
     * <p>
     * This method creates instances of the swerve modules using reflection based on the provided class type and configurations.
     * It also ensures that necessary configurations, such as the presence of absolute encoder names, are properly set.
     *
     * @param <T>            The type of swerve module to be instantiated.
     * @param clazz          The class of the swerve module to be created.
     * @param configurations An array of configurations for each swerve module.
     * @throws NullPointerException if an expected configuration value (e.g., absolute encoder name) is missing.
     */
    public <T extends SwerveModule> void init(Class<T> clazz, SwerveModuleConfiguration[] configurations) {
        try {
            if (clazz == AxonSwerveModule.class) {
                for (SwerveModuleConfiguration configuration : configurations) {
                    if (configuration.absoluteEncoderName == null) {
                        throw new NullPointerException("Expected absolute encoder name for axon but got null!");
                    }
                }
            }

            m_fr = clazz.getDeclaredConstructor(SwerveModuleConfiguration.class, SwerveDriveCoefficients.class, HardwareMap.class).newInstance(configurations[0], m_coefficients, m_hardwareMap);
            m_bl = clazz.getDeclaredConstructor(SwerveModuleConfiguration.class, SwerveDriveCoefficients.class, HardwareMap.class).newInstance(configurations[1], m_coefficients, m_hardwareMap);
            m_fl = clazz.getDeclaredConstructor(SwerveModuleConfiguration.class, SwerveDriveCoefficients.class, HardwareMap.class).newInstance(configurations[2], m_coefficients, m_hardwareMap);
            m_br = clazz.getDeclaredConstructor(SwerveModuleConfiguration.class, SwerveDriveCoefficients.class, HardwareMap.class).newInstance(configurations[3], m_coefficients, m_hardwareMap);

            initIMU(m_coefficients.imuName);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    /**
     * Initializes the IMU.
     */
    private void initIMU(String imuName) {
        m_imu = m_hardwareMap.get(BNO055IMU.class, imuName);

        m_imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        m_imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        RobotLog.d("imu params init start");
        m_imu.initialize(m_imuParameters);
        RobotLog.d("imu init finished");

        m_telemetry.addData("Gyro", "calibrating...");

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!m_imu.isGyroCalibrated() && !m_robot.isStopRequested() && timer.seconds() < 5) {
            m_robot.sleep(50);
        }
        if (m_imu.isGyroCalibrated()) {
            m_robot.telemetry.addData("Gyro", "Done Calibrating");
            RobotLog.d("Gyro done init");
            m_odometry = new SwerveDriveOdometry(m_kinematics, getHeadingRotation2d());
        } else {
            m_robot.telemetry.addData("Gyro", "Gyro/IMU Calibration Failed");
            RobotLog.d("Gyro failed init" + " " + m_imu.isGyroCalibrated() + " " + m_imu.isAccelerometerCalibrated() + " " + m_imu.isMagnetometerCalibrated());
        }
    }

    /**
     * Sets the power for the swerve drive system.
     *
     * @param x    the x component of the desired velocity
     * @param y    the y component of the desired velocity
     * @param turn the rotational component of the desired velocity
     */
    @Override
    public void setPower(double x, double y, double turn) {
        // Create a ChassisSpeeds object with the desired velocities
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(x, y, turn);

        // Convert the ChassisSpeeds to SwerveModuleStates
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize the speeds to make sure no wheel speed exceeds 1.0
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, 1.0);

        // Set the powers to the modules
        m_fl.setPower(moduleStates[0].speedMetersPerSecond, moduleStates[0].angle.getDegrees());
        m_fr.setPower(moduleStates[1].speedMetersPerSecond, moduleStates[1].angle.getDegrees());
        m_bl.setPower(moduleStates[2].speedMetersPerSecond, moduleStates[2].angle.getDegrees());
        m_br.setPower(moduleStates[3].speedMetersPerSecond, moduleStates[3].angle.getDegrees());
    }

    /**
     * Updates the odometry for the swerve drive system.
     */
    public void update() {
        updateOdometry();
    }

    /**
     * Sets the power for the swerve drive system with an option for field-oriented control.
     *
     * @param x             the x component of the desired velocity
     * @param y             the y component of the desired velocity
     * @param turn          the rotational component of the desired velocity
     * @param fieldOriented whether the control should be field-oriented
     */
    @Override
    public void setPowerOriented(double x, double y, double turn, boolean fieldOriented) {
        if (!fieldOriented) {  // No field oriented (=> Robot oriented)
            setPower(x, y, turn);
        } else {
            double phiRad = (-getHeading() + m_angleOffset) / 180 * Math.PI;
            double forward = y * Math.cos(phiRad) - x * Math.sin(phiRad);
            double strafe = y * Math.sin(phiRad) + x * Math.cos(phiRad);
            setPower(forward, turn, strafe);
        }
    }

    /**
     * Gets the current x position of the robot.
     *
     * @return the current x position
     */
    @Override
    public double getPosX() {
        return m_posX;
    }

    /**
     * Gets the current y position of the robot.
     *
     * @return the current y position
     */
    @Override
    public double getPosY() {
        return m_posY;
    }

    /**
     * Gets the current heading of the robot.
     *
     * @return the current heading in degrees
     */
    @Override
    public double getHeading() {
        Orientation orientation = m_imu.getAngularOrientation();
        return (-orientation.firstAngle + m_startingPosition.angle) % 360;
    }

    /**
     * Gets the current heading as a Rotation2d object.
     *
     * @return the current heading as a Rotation2d object
     */
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Resets the orientation of the robot.
     *
     * @param angle the angle to set as the new orientation
     */
    @Override
    public void resetOrientation(double angle) {
        m_imu.initialize(m_imuParameters);
        m_angleOffset = angle;
    }

    /**
     * Updates the odometry for the swerve drive system.
     */
    private void updateOdometry() {
        SwerveModuleState[] moduleStates = {m_fl.getState(), m_fr.getState(), m_bl.getState(), m_br.getState()};

        m_odometry.updateWithTime(System.currentTimeMillis() * 1000, getHeadingRotation2d(), moduleStates);

        m_posX = m_odometry.getPoseMeters().getX();
        m_posY = m_odometry.getPoseMeters().getY();
    }

    /**
     * Moves the robot to a specified location with specified settings and an optional midway action.
     *
     * @param location     The target location.
     * @param settings     The settings for the movement.
     * @param midwayAction An optional action to perform midway through the movement.
     */
    @Override
    public void goToLocation(Location location, BaseDrive.GotoSettings settings, Runnable midwayAction) {
        // Determine the relative position of the target location
        double targetX = location.x - m_posX;
        double targetY = location.y - m_posY;

        // Calculate the angle to the target location
        double targetAngle = Math.toDegrees(Math.atan2(targetY, targetX));

        // Calculate the distance to the target location
        double distance = Math.hypot(targetX, targetY);

        // Call the goTo method to move the robot to the location
        double remainingDistance = goToLocationInternal(location.x, location.y, settings.power, targetAngle, settings.tolerance, settings.timeout, settings.noSlowdown, midwayAction, null);

        // Check if the movement was successful
        if (remainingDistance < settings.tolerance) {
            // Movement succeeded
            // Add any success handling here
        } else {
            // Movement failed
            // Add any failure handling here
        }
    }

    /**
     * Moves the robot to a specified location with specified settings.
     *
     * @param location The target location.
     * @param settings The settings for the movement.
     */
    @Override
    public void goToLocation(Location location, GotoSettings settings) {
        goToLocation(location, settings, null);
    }

    /**
     * Internal method to perform the movement to a specified location.
     *
     * @param x             The x-coordinate of the target location.
     * @param y             The y-coordinate of the target location.
     * @param power         The power for the movement.
     * @param targetHeading The target heading (angle) of the robot.
     * @param tolerance     The tolerance for considering the robot has reached the target.
     * @param timeout       The timeout for the movement.
     * @param noSlowdown    Whether to apply slowdown.
     * @param midwayAction  An optional action to perform midway through the movement.
     * @param onFinish      An optional action to perform when the movement finishes.
     * @return The remaining distance to the target after the movement.
     */
    private double goToLocationInternal(double x, double y, double power, double targetHeading, double tolerance, double timeout, boolean noSlowdown, Runnable midwayAction, Runnable onFinish) {
        int goToIdle = 0; //if not moving
        int stuckTries = 0;
        boolean isCheckingIdle = false;
        Location currentTarget = new Location(x, y, targetHeading);
        GotoSettings currentSettings = new GotoSettings.Builder().setPower(power).setTolerance(tolerance).setTimeout(timeout).setSlowdownMode(!noSlowdown).build();
        Location startTarget = currentTarget;
        double lastX = 0;
        double lastY = 0;

        double currentX = getPosX();
        double currentY = getPosY();
        double deltaX = currentTarget.x - currentX;
        double deltaY = currentTarget.y - currentY;
        double startX = currentX;
        double startY = currentY;
        double remainDist = 0;

        Location returnToPosAfterStuck = new Location(startX, startY);
        double totalDist = Math.hypot(deltaX, deltaY);
        double currentDist = 0;

        ElapsedTime timer = new ElapsedTime();

        while (m_robot.opModeIsActive() && (currentDist < (totalDist - currentSettings.tolerance)) && !m_robot.isStopRequested() && stuckTries < 15) {
            double powerToUse = currentSettings.power;

            currentX = getPosX();
            currentY = getPosY();
            deltaX = currentX - startX;
            deltaY = currentY - startY;
            currentDist = Math.hypot(deltaY, deltaX); // distance moved from start position.
            deltaX = currentTarget.x - currentX;
            deltaY = currentTarget.y - currentY;
            remainDist = Math.hypot(deltaY, deltaX);  // distance left to target.

            double minPower = 0.2;

            double acclGain = currentSettings.noSlowdown ? 3.0 : 1.5;
            double acclPower = currentDist * acclGain + minPower;

            if (acclPower < powerToUse) {
                powerToUse = acclPower;
            }

            double breakGain = currentSettings.noSlowdown ? 1.0 : 0.5;
            double breakPower = remainDist * breakGain + minPower;

            if (breakPower < powerToUse) {
                powerToUse = breakPower;
            }

            double headingErr = getDeltaHeading(currentTarget.angle) / 180;
            double headingGain = Math.max(0.6, -0.25 * totalDist + 0.95); // y = -0.25x + 0.95
            double correction = headingGain * headingErr;

            // Set power to the swerve modules
            setPower(powerToUse, correction, 0);

            // Perform midway action if provided
            if (midwayAction != null) {
                midwayAction.run();
            }

            // Check if timeout has occurred
            if ((currentSettings.timeout != 0 && currentSettings.timeout <= timer.seconds())) {
                break;
            }

            // Idle checker
            double velocityRange = 0.0001;
            double dx = lastX - currentX;
            double dy = lastY - currentY;
            double velocity = Math.hypot(dx, dy) / timer.seconds();

            if (remainDist < 0.25 && Math.abs(velocity) < velocityRange) {
                goToIdle += 1;
            }

            if (Math.abs(currentX - lastX) < 0.02 && Math.abs(velocity) < velocityRange) {
                stuckTries += 1;
                returnToPosAfterStuck = new Location(getPosX(), getPosY());
            }

            lastX = currentX;
            lastY = currentY;
            if (goToIdle >= MAX_IDLE_BREAK) {
                remainDist = -1;
                break;
            }
        }

        // Run onFinish action if provided
        if (onFinish != null) {
            onFinish.run();
        }

        // Return remaining distance to target
        return remainDist;
    }
}
