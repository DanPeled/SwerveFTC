package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.util.ECSSystem.BaseDrive;
import org.firstinspires.ftc.teamcode.util.ECSSystem.Robot;
import org.firstinspires.ftc.teamcode.util.Location;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Represents a sequence of steps to navigate through during autonomous mode.
 * This class provides methods to execute the path step by step or all at once.
 */
public class AutoPath {
    private static BaseDrive drive;
    private static Location prevLocation;
    private final ArrayList<Marker> path;
    public Location startLocation;
    private int currentLocationIndex = 0;

    /**
     * Constructs an AutoPath object with the specified drive system, start location, and steps.
     *
     * @param robot         The robot instance to control robot movement.
     * @param startLocation The starting location of the robot.
     * @param waypoints     The list of steps defining the path.
     * @see Waypoint
     */
    private AutoPath(Robot robot, Location startLocation, Waypoint... waypoints) {
        this.path = new ArrayList<>();
        path.addAll(Arrays.asList(waypoints));
        drive = robot.getComponent(BaseDrive.class);
        this.startLocation = startLocation;
        prevLocation = startLocation;
    }

    /**
     * Executes a specified number of steps in the path.
     *
     * @param steps The number of steps to execute.
     * @throws RuntimeException if steps are negative.
     */
    public void step(int steps) {
        if (steps == 0) return;

        if (steps < 0) {
            throw new RuntimeException("Cannot go negative amount of steps");
        } else {
            for (int i = 0; i < steps; i++) {
                Marker current = path.get(currentLocationIndex);
                if (path.get(currentLocationIndex) instanceof Waypoint) {
                    Waypoint currentWaypoint = (Waypoint) current;
                    current.run();
                    if (currentWaypoint.type == Waypoint.Type.Step)
                        prevLocation = prevLocation.add(currentWaypoint.locationDelta);
                    else {
                        prevLocation = currentWaypoint.locationDelta;
                    }
                }
                skip();
            }
        }
    }

    /**
     * Executes one step in the path.
     */
    public void step() {
        step(1);
    }

    /**
     * Skips a specified number of steps in the path.
     *
     * @param amount The number of steps to skip.
     */
    public void skip(int amount) {
        currentLocationIndex += amount;
    }

    /**
     * Skips one waypoint in the path.
     */
    public void skip() {
        skip(1);
    }

    /**
     * Executes the remaining steps in the path.
     */
    public void run() {
        step(path.size() - currentLocationIndex);
    }

    /**
     * Flips the Y axis of all steps in the path.
     */
    public void flipY() {
        for (int i = 0; i < path.size(); i++) {
            Marker marker = path.get(i);
            if (marker instanceof Waypoint) {
                Waypoint newW = (Waypoint) marker;
                if (newW.type == Waypoint.Type.StaticWaypoint) continue;
                newW.locationDelta.flipY();
                path.set(i, newW);
            }
        }
    }

    /**
     * Flips the X axis of all steps in the path.
     */
    public void flipX() {
        for (Waypoint waypoint : path.toArray(new Waypoint[0])) {
            waypoint.locationDelta.flipX();
        }
    }

    /**
     * Represents a single waypoint in the path.
     */
    public static class Waypoint extends Marker {
        public Runnable midwayAction;
        public Location locationDelta;
        public BaseDrive.GotoSettings settings;
        public Type type = Type.Step;

        /**
         * Constructs a Waypoint object with the specified location and movement settings.
         *
         * @param locationDelta The target location of the waypoint.
         * @param settings      The movement settings for reaching the target location.
         * @param type          The type of waypoint.
         * @see Location
         * @see org.firstinspires.ftc.teamcode.util.ECSSystem.BaseDrive.GotoSettings
         */
        public Waypoint(Location locationDelta, BaseDrive.GotoSettings settings, Type type) {
            this(locationDelta, settings, type, null);
        }

        public Waypoint(Location locationDelta, BaseDrive.GotoSettings settings, Type type, Runnable midwayAction) {
            this.locationDelta = locationDelta;
            this.settings = settings;
            this.type = type;
            this.midwayAction = midwayAction;
            this.runFunction = this::runWaypoint;
        }

        /**
         * Executes movement to reach the waypoint's location using the specified settings.
         */
        public void runWaypoint() {
            Location location = locationDelta;
            if (locationDelta.x == 0 && locationDelta.y == 0) {
                drive.turnTo(locationDelta.angle, settings.power / 2);
                return;
            }
            if (type == Type.Step) {
                location = prevLocation.add(locationDelta);
                location.angle = locationDelta.angle;
            } else if (type == Type.Location) {
                prevLocation = location;
            }
            if (midwayAction != null) drive.goToLocation(location, settings, midwayAction);
            else drive.goToLocation(location, settings);
        }

        /**
         * Enumerates the type of waypoint.
         */
        public enum Type {
            Step, Location, StaticWaypoint
        }
    }

    /**
     * Builder class for constructing AutoPath objects.
     */
    public static class Builder {
        private final ArrayList<Marker> steps = new ArrayList<>();

        public Builder addStep(Location location, BaseDrive.GotoSettings settings, Runnable midwayAction) {
            return addWaypointInternal(location, settings, Waypoint.Type.Step, midwayAction);
        }

        /**
         * Adds a waypoint to the path with the specified movement settings.
         *
         * @param location The target location of the waypoint.
         * @param settings The movement settings for reaching the target location.
         * @return The {@link Builder} instance.
         * @see Location
         */
        public Builder addStep(Location location, BaseDrive.GotoSettings settings) {
            return addStep(location, settings, null);
        }


        /**
         * Adds a location to the path with the specified location settings.
         *
         * @param location The target location of the waypoint.
         * @param settings The movement settings for reaching the target location.
         * @return The {@link Builder} instance.
         * @see Location
         */
        public Builder addLocation(Location location, BaseDrive.GotoSettings settings) {
            return addWaypointInternal(location, settings, Waypoint.Type.Location);
        }

        /**
         * Adds a static waypoint (isn't getting flipped) to the path with the specified location settings
         *
         * @param location The target location of the waypoint
         * @param settings The movement settings for reaching the target location
         * @return The {@link Builder} instance.
         * @see Location
         */
        public Builder addStaticWaypoint(Location location, BaseDrive.GotoSettings settings) {
            return addWaypointInternal(location, settings, Waypoint.Type.StaticWaypoint);
        }

        /**
         * Adds a new waypoint with the specified type, location and settings
         *
         * @param location The target location
         * @param settings Movement settings for reaching the target location
         * @param type     The waypoint type : Step (A movement), Location (a specific place) or a Static Waypoint (a specific place that isn't flipped together with the rest of the path.
         * @return The {@link Builder} instance.
         * @see #addLocation(Location, BaseDrive.GotoSettings)
         * @see #addStep(Location, BaseDrive.GotoSettings)
         * @see #addStaticWaypoint(Location, BaseDrive.GotoSettings)
         */
        private Builder addWaypointInternal(Location location, BaseDrive.GotoSettings settings, Waypoint.Type type, Runnable midwayAction) {
            steps.add(new Waypoint(location, settings, type, midwayAction));
            return this;
        }

        private Builder addWaypointInternal(Location location, BaseDrive.GotoSettings settings, Waypoint.Type type) {
            return addWaypointInternal(location, settings, type, null);
        }

        public Builder addAction(Runnable action) {
            steps.add(new Marker() {
                @Override
                public void run() {
                    action.run();
                }
            });
            return this;
        }

        /**
         * Constructs an AutoPath object with the specified drive system and start location.
         *
         * @param robot         The {@link Robot} instance to control robot movement.
         * @param startLocation The starting location of the robot.
         * @return The constructed AutoPath object.
         */
        public AutoPath build(Robot robot, Location startLocation) {
            try {
                return new AutoPath(robot, startLocation, steps.toArray(new Waypoint[0]));
            } catch (Exception e) {
                return null;
            }
        }
    }

    public static abstract class Marker {
        Runnable runFunction;

        public void run() {
            runFunction.run();
        }
    }
}
