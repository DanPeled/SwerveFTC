package com.danpeled.swerveftclib.examples;

import com.danpeled.swerveftclib.util.BaseDrive;
import com.danpeled.swerveftclib.util.CommandUtil.CommandFor;
import com.danpeled.swerveftclib.util.Location;

import java.util.function.Supplier;

public class SwerveCommands {

    /**
     * Command to set the power of the swerve drive in an oriented manner.
     */
    public static class SetPowerOriented extends CommandFor<ExampleSwerveSubsystem> {
        private final Supplier<Double> m_supplierX, m_supplierY, m_supplierTurn;
        private final boolean fieldOriented;

        /**
         * Constructs a new SetPowerOriented command.
         *
         * @param swerveDrive   The swerve drive subsystem.
         * @param supplierX     The supplier for the X-axis power.
         * @param supplierY     The supplier for the Y-axis power.
         * @param supplierTurn  The supplier for the turning power.
         * @param fieldOriented Whether the control is field-oriented.
         */
        public SetPowerOriented(ExampleSwerveSubsystem swerveDrive, Supplier<Double> supplierX, Supplier<Double> supplierY, Supplier<Double> supplierTurn, boolean fieldOriented) {
            super(swerveDrive);
            this.fieldOriented = fieldOriented;
            this.m_supplierTurn = supplierTurn;
            this.m_supplierX = supplierX;
            this.m_supplierY = supplierY;
        }

        /**
         * Executes the command to set the power of the swerve drive based on supplier input.
         */
        @Override
        public void execute() {
            double x = m_supplierX.get();
            double y = m_supplierY.get();
            double turn = m_supplierTurn.get();

            if (Math.abs(x) < 0.1) x = 0;
            if (Math.abs(y) < 0.1) y = 0;
            if (Math.abs(turn) < 0.1) turn = 0;

            subsystem.getDrive().setPowerOriented(x, y, turn, fieldOriented);
        }
    }

    /**
     * Command to move the swerve drive to a specific location.
     */
    public static class GoTo extends CommandFor<ExampleSwerveSubsystem> {
        private final Location m_location;
        private final BaseDrive.GotoSettings m_config;
        private final Runnable m_midwayAction;

        /**
         * Constructs a new GoTo command.
         *
         * @param swerveDrive  The swerve drive subsystem.
         * @param location     The target location to move to.
         * @param config       The configuration settings for the movement.
         * @param midwayAction An optional action to perform midway through the movement.
         */
        public GoTo(ExampleSwerveSubsystem swerveDrive, Location location, BaseDrive.GotoSettings config, Runnable midwayAction) {
            super(swerveDrive);
            this.m_location = location;
            this.m_config = config;
            this.m_midwayAction = midwayAction;
        }

        /**
         * Constructs a new GoTo command without a midway action.
         *
         * @param swerveDrive The swerve drive subsystem.
         * @param location    The target location to move to.
         * @param config      The configuration settings for the movement.
         */
        public GoTo(ExampleSwerveSubsystem swerveDrive, Location location, BaseDrive.GotoSettings config) {
            this(swerveDrive, location, config, null);
        }

        /**
         * Executes the command to move the swerve drive to the specified location.
         */
        @Override
        public void execute() {
            subsystem.getDrive().goToLocation(m_location, m_config, m_midwayAction);
        }
    }

    /**
     * Command to turn the swerve drive to a specific angle.
     */
    public static class TurnTo extends CommandFor<ExampleSwerveSubsystem> {
        private final double m_angle;
        private final double m_power;

        /**
         * Constructs a new TurnTo command.
         *
         * @param swerveDrive The swerve drive subsystem.
         * @param angle       The target angle to turn to.
         * @param power       The power to apply while turning.
         */
        public TurnTo(ExampleSwerveSubsystem swerveDrive, double angle, double power) {
            super(swerveDrive);
            this.m_angle = angle;
            this.m_power = power;
        }

        /**
         * Executes the command to turn the swerve drive to the specified angle.
         */
        @Override
        public void execute() {
            subsystem.getDrive().turnTo(m_angle, m_power);
        }
    }
}
