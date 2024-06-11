package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Swerve.SwerveDrive;

/**
 * SampleDrive class represents a sample teleoperated mode for controlling a swerve drive robot.
 * It extends the Robot class and implements the necessary methods for initializing and running the robot.
 */
@TeleOp
public class SampleDrive extends CommandOpMode {
    SwerveDrive swerveDrive;

    @Override
    public void initialize() {
        swerveDrive = new SwerveDrive(this);
    }
}
