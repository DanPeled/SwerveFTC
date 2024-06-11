package com.danpeled.swerveftclib.util;

/**
 * Represents a location in a 2D coordinate system with an optional angle.
 */
public class Location {

    /**
     * The x-coordinate of the location.
     */
    public double x;

    /**
     * The y-coordinate of the location.
     */
    public double y;

    /**
     * The angle of the location (default is 0).
     */
    public double angle = 0;

    /**
     * Constructs a location with specified x and y coordinates.
     *
     * @param x The x-coordinate.
     * @param y The y-coordinate.
     */
    public Location(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Constructs a location with specified x, y coordinates, and angle.
     *
     * @param x     The x-coordinate.
     * @param y     The y-coordinate.
     * @param angle The angle in degrees.
     */
    public Location(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    /**
     * Constructs a location as a copy of another location.
     *
     * @param location The location to copy.
     */
    public Location(Location location) {
        this.x = location.x;
        this.y = location.y;
        this.angle = location.angle;
    }

    /**
     * Constructs a location as an offset of another location.
     *
     * @param location    The base location.
     * @param xOffset     The x offset.
     * @param yOffset     The y offset.
     * @param angleOffset The angle offset.
     */
    public Location(Location location, double xOffset, double yOffset, double angleOffset) {
        this.x = location.x + xOffset;
        this.y = location.y + yOffset;
        this.angle = location.angle + angleOffset;
    }

    /**
     * Flips the x-coordinate of the location.
     */
    public void flipX() {
        this.x *= -1;
    }

    /**
     * Flips the y-coordinate of the location.
     *
     * @return The updated location object.
     */
    public Location flipY() {
        this.y *= -1;
        return this;
    }

    /**
     * Flips the angle of the location.
     */
    public void flipAngle() {
        this.angle *= -1;
    }

    /**
     * Creates a new location with an offset in the y-coordinate.
     *
     * @param offset The y-offset.
     * @return The new location.
     */
    public Location offsetY(double offset) {
        return new Location(this.x, this.y + offset, this.angle);
    }

    /**
     * Adds another location to this location and returns the result.
     *
     * @param location The location to add.
     * @return The new location with added coordinates.
     */
    public Location add(Location location) {
        Location result = this;
        result = result.addX(location.x);
        result = result.addY(location.y);
        return result;
    }

    /**
     * Adds an offset to the x-coordinate and returns the new location.
     *
     * @param x The x-offset.
     * @return The new location.
     */
    public Location addX(double x) {
        return new Location(this.x + x, this.y, this.angle);
    }

    /**
     * Adds an offset to the y-coordinate and returns the new location.
     *
     * @param y The y-offset.
     * @return The new location.
     */
    public Location addY(double y) {
        return new Location(this.x, this.y + y, this.angle);
    }

    /**
     * Subtracts another location from this location and returns the result.
     *
     * @param location The location to subtract.
     * @return The new location with subtracted coordinates.
     */
    public Location subtract(Location location) {
        Location result = this;
        result = result.subtractX(location.x);
        result = result.subtractY(location.y);
        return result;
    }

    /**
     * Subtracts an offset from the x-coordinate and returns the new location.
     *
     * @param x The x-offset.
     * @return The new location.
     */
    public Location subtractX(double x) {
        return this.addX(-x);
    }

    /**
     * Subtracts an offset from the y-coordinate and returns the new location.
     *
     * @param y The y-offset.
     * @return The new location.
     */
    public Location subtractY(double y) {
        return this.addY(-y);
    }

    /**
     * Creates a new location with an offset in the x-coordinate.
     *
     * @param offset The x-offset.
     * @return The new location.
     */
    public Location offsetX(double offset) {
        return new Location(this.x + offset, this.y, this.angle);
    }
}
