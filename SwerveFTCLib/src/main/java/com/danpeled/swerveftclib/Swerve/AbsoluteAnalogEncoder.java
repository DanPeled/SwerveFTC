package com.danpeled.swerveftclib.Swerve;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * A class that interfaces with an analog encoder to provide absolute angle measurements.
 * The encoder is used to read the voltage from an analog input, which is then converted to an angle.
 */
@Config
public class AbsoluteAnalogEncoder {

    /**
     * The default range of the analog input in volts.
     */
    public static double DEFAULT_RANGE = 3.3;

    /**
     * Flag to enable or disable value rejection for noise filtering.
     */
    public static boolean VALUE_REJECTION = false;

    /**
     * The analog input hardware used to read the encoder value.
     */
    private final AnalogInput encoder;

    /**
     * The offset to be applied to the encoder readings.
     */
    private double offset;

    /**
     * The range of the analog input in volts.
     */
    private final double analogRange;

    /**
     * Flag indicating if the encoder values should be inverted.
     */
    private boolean inverted;

    /**
     * The last recorded position to help with value rejection.
     */
    private double pastPosition = 1;

    /**
     * Constructs an AbsoluteAnalogEncoder with the default range.
     *
     * @param enc The analog input used for the encoder.
     */
    public AbsoluteAnalogEncoder(AnalogInput enc) {
        this(enc, DEFAULT_RANGE);
    }

    /**
     * Constructs an AbsoluteAnalogEncoder with a specified range.
     *
     * @param enc    The analog input used for the encoder.
     * @param aRange The range of the analog input in volts.
     */
    public AbsoluteAnalogEncoder(AnalogInput enc, double aRange) {
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = false;
    }

    /**
     * Sets the offset for the encoder readings.
     *
     * @param off The offset value to be set.
     * @return The current instance of AbsoluteAnalogEncoder.
     */
    public AbsoluteAnalogEncoder zero(double off) {
        offset = off;
        return this;
    }

    /**
     * Sets whether the encoder readings should be inverted.
     *
     * @param invert True to invert the readings, false otherwise.
     * @return The current instance of AbsoluteAnalogEncoder.
     */
    public AbsoluteAnalogEncoder setInverted(boolean invert) {
        inverted = invert;
        return this;
    }

    /**
     * Gets whether the encoder readings are inverted.
     *
     * @return True if the readings are inverted, false otherwise.
     */
    public boolean getDirection() {
        return inverted;
    }

    /**
     * Gets the current position of the encoder in radians, taking into account inversion and offset.
     *
     * @return The current position of the encoder in radians.
     */
    public double getCurrentPosition() {
        double pos = Angle.norm((!inverted ? 1 - getVoltage() / analogRange : getVoltage() / analogRange) * Math.PI * 2 - offset);
        // Checks for anomalous values when the encoder is close to zero
        if (!VALUE_REJECTION || Math.abs(Angle.normDelta(pastPosition)) > 0.1 || Math.abs(Angle.normDelta(pos)) < 1) {
            pastPosition = pos;
        }
        return pastPosition;
    }

    /**
     * Gets the analog input hardware associated with this encoder.
     *
     * @return The analog input used by the encoder.
     */
    public AnalogInput getEncoder() {
        return encoder;
    }

    /**
     * Gets the current voltage reading from the analog input.
     *
     * @return The voltage reading from the analog input.
     */
    public double getVoltage() {
        return encoder.getVoltage();
    }
}
