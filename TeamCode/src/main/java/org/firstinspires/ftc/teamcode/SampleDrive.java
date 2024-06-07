package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.util.ECSSystem.Robot;

/**
 * SampleDrive class represents a sample teleoperated mode for controlling a swerve drive robot.
 * It extends the Robot class and implements the necessary methods for initializing and running the robot.
 */
@TeleOp
public class SampleDrive extends Robot {
    // Instance of the SwerveDrive system
    SwerveDrive swerveDrive;

    /**
     * Initializes the robot and adds the SwerveDrive component.
     * This method is called once when the INIT button is pressed.
     */
    @Override
    public void initRobot() {
        swerveDrive = addComponent(SwerveDrive.class);
    }

    /**
     * Starts the robot. This method is called once when the PLAY button is pressed.
     */
    @Override
    public void startRobot() {
    }

    /**
     * Updates the robot's control loop. This method is called repeatedly while the robot is running.
     * It sets the power to the swerve drive system based on the gamepad inputs.
     */
    @Override
    public void updateLoop() {
        // Set power to the swerve drive using the gamepad inputs.
        // The left stick controls the translation (x and y movement),
        // and the right stick controls the rotation (turn).
        swerveDrive.setPowerOriented(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, true);
    }
}
