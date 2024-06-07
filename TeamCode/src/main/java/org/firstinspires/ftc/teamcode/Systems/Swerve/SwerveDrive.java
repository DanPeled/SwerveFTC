package org.firstinspires.ftc.teamcode.Systems.Swerve;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.ECSSystem.BaseDrive;
import org.firstinspires.ftc.teamcode.util.Location;

/**
 * SwerveDrive class controls a swerve drive system for an FTC robot.
 */
public class SwerveDrive extends BaseDrive {
    /**
     * Maximum number of idle breaks before considering the robot stopped.
     */
    public final int MAX_IDLE_BREAK = 20;

    /**
     * Length of the robot (distance between front and back wheels).
     */
    public final double LENGTH = 1;

    /**
     * Width of the robot (distance between left and right wheels).
     */
    public final double WIDTH = 1;

    /**
     * Parameters for the IMU (Inertial Measurement Unit).
     */
    private final BNO055IMU.Parameters m_imuParameters = new BNO055IMU.Parameters();

    /**
     * Starting position of the robot.
     */
    private final Location startingPosition = new Location(0, 0);

    /**
     * Instance of the IMU.
     */
    private BNO055IMU imu = null;

    /**
     * Swerve modules of the robot.
     */
    private SwerveModule m_fl = null, m_fr = null, m_bl = null, m_br = null;

    /**
     * Angle offset for the robot's orientation.
     */
    private double angleOffset = 0;

    /**
     * Current X position of the robot.
     */
    private double posX = 0;

    /**
     * Current Y position of the robot.
     */
    private double posY = 0;

    /**
     * Swerve Drive Kinematics for the robot.
     */
    private SwerveDriveKinematics kinematics;

    /**
     * Swerve Drive Odometry for the robot.
     */
    private SwerveDriveOdometry odometry;

    /**
     * Initializes the swerve drive system.
     */
    @Override
    public void init() {
        m_fl = new SwerveModule("fl", "flServo", hardwareMap);
        m_fr = new SwerveModule("fr", "frServo", hardwareMap);
        m_bl = new SwerveModule("bl", "blServo", hardwareMap);
        m_br = new SwerveModule("br", "brServo", hardwareMap);

        initIMU(hardwareMap);

        kinematics = new SwerveDriveKinematics(new Translation2d(LENGTH / 2, WIDTH / 2), new Translation2d(LENGTH / 2, -WIDTH / 2), new Translation2d(-LENGTH / 2, WIDTH / 2), new Translation2d(-LENGTH / 2, -WIDTH / 2));

        odometry = new SwerveDriveOdometry(kinematics, getHeadingRotation2d());
    }

    /**
     * Initializes the IMU.
     *
     * @param hardwareMap the hardware map
     */
    private void initIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");

        m_imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        m_imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        RobotLog.d("imu params init start");
        imu.initialize(m_imuParameters);
        RobotLog.d("imu init finished");

        robot.telemetry.addData("Gyro", "calibrating...");

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!imu.isGyroCalibrated() && !robot.isStopRequested() && timer.seconds() < 5) {
            robot.sleep(50);
        }
        if (imu.isGyroCalibrated()) {
            robot.telemetry.addData("Gyro", "Done Calibrating");
            RobotLog.d("Gyro done init");
        } else {
            robot.telemetry.addData("Gyro", "Gyro/IMU Calibration Failed");
            RobotLog.d("Gyro failed init" + " " + imu.isGyroCalibrated() + " " + imu.isAccelerometerCalibrated() + " " + imu.isMagnetometerCalibrated());
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
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

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
    @Override
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
            double phiRad = (-getHeading() + angleOffset) / 180 * Math.PI;
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
        return posX;
    }

    /**
     * Gets the current y position of the robot.
     *
     * @return the current y position
     */
    @Override
    public double getPosY() {
        return posY;
    }

    /**
     * Gets the current heading of the robot.
     *
     * @return the current heading in degrees
     */
    @Override
    public double getHeading() {
        Orientation orientation = imu.getAngularOrientation();
        return (-orientation.firstAngle + startingPosition.angle) % 360;
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
        imu.initialize(m_imuParameters);
        angleOffset = angle;
    }

    /**
     * Updates the odometry for the swerve drive system.
     */
    private void updateOdometry() {
        SwerveModuleState[] moduleStates = {m_fl.getState(), m_fr.getState(), m_bl.getState(), m_br.getState()};

        odometry.updateWithTime(System.currentTimeMillis() * 1000, getHeadingRotation2d(), moduleStates);

        posX = odometry.getPoseMeters().getX();
        posY = odometry.getPoseMeters().getY();
    }

    /**
     * Moves the robot to a specified location with specified settings and an optional midway action.
     *
     * @param location     The target location.
     * @param settings     The settings for the movement.
     * @param midwayAction An optional action to perform midway through the movement.
     */
    @Override
    public void goToLocation(Location location, GotoSettings settings, Runnable midwayAction) {
        // Determine the relative position of the target location
        double targetX = location.x - posX;
        double targetY = location.y - posY;

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

        while (robot.opModeIsActive() && (currentDist < (totalDist - currentSettings.tolerance)) && !robot.isStopRequested() && stuckTries < 15) {
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
