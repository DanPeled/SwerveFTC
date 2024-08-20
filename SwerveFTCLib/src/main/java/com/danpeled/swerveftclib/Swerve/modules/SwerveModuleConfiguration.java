package com.danpeled.swerveftclib.Swerve.modules;

import com.arcrobotics.ftclib.controller.PIDFController;

public class SwerveModuleConfiguration {
    public final String driveMotorName, angleServoName, absoluteEncoderName;
    public PIDFController drivePIDFController, anglePIDFController;


    public SwerveModuleConfiguration(PIDFController drivePIDFController, PIDFController anglePIDFController, String driveMotorName, String angleServoName, String absoluteEncoderName) {
        this.drivePIDFController = drivePIDFController;
        this.anglePIDFController = anglePIDFController;
        this.driveMotorName = driveMotorName;
        this.angleServoName = angleServoName;
        this.absoluteEncoderName = absoluteEncoderName;
    }

    public static SwerveModuleConfiguration create(String driveMotorName, String angleServoName, String absoluteEncoderName) {
        return new SwerveModuleConfiguration(
                null,
                null,
                driveMotorName,
                angleServoName,
                absoluteEncoderName
        );
    }
}
