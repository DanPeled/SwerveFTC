package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Swerve.SwerveCommands;
import org.firstinspires.ftc.teamcode.Systems.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.util.BaseDrive;
import org.firstinspires.ftc.teamcode.util.Location;

/**
 * SampleDrive class represents a sample teleoperated mode for controlling a swerve drive robot.
 * It extends the CommandOpMode class and implements the necessary methods for initializing and running the robot.
 *
 * @see CommandOpMode
 */
@TeleOp
public class SampleDrive extends CommandOpMode {
    SwerveDrive swerveDrive;
    GamepadEx driver;

    /**
     * Initializes the teleoperated mode by setting up the swerve drive subsystem,
     * configuring the default command, and mapping the gamepad buttons to commands.
     */
    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        swerveDrive = new SwerveDrive(this);

        // Set the default command for the swerve drive to SetPowerOriented
        swerveDrive.setDefaultCommand(new SwerveCommands.SetPowerOriented(swerveDrive, gamepad1, true));

        // Map the A button on the gamepad to the GoTo command
        Button exampleButton = new GamepadButton(driver, GamepadKeys.Button.A);

        // Configure default settings for the GoTo command
        BaseDrive.GotoSettings defaultGoToSettings = new BaseDrive.GotoSettings.Builder()
                .setPower(1)
                .setSlowdownMode(false)
                .setTolerance(0.5)
                .build();

        // Assign the GoTo command to the A button
        exampleButton.whenPressed(new SwerveCommands.GoTo(swerveDrive, new Location(0, 0), defaultGoToSettings));

        // Register the swerve drive subsystem
        register(swerveDrive);
    }
}
