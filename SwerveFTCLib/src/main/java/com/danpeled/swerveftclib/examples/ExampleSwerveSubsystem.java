package com.danpeled.swerveftclib.examples;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.danpeled.swerveftclib.Swerve.SwerveDrive;

public class ExampleSwerveSubsystem extends SubsystemBase {
    private final SwerveDrive m_swerveDrive;

    public ExampleSwerveSubsystem(SwerveDrive drive) {
        this.m_swerveDrive = drive;
    }

    @Override
    public void periodic() {
        this.m_swerveDrive.update();
    }

    public SwerveDrive getDrive() {
        return this.m_swerveDrive;
    }
}
