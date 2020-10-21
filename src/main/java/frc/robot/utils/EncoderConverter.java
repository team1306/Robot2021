package frc.robot.utils;

/**
 * A class for storing the encoder constants and mathematics around translating
 * encoder values into physical values.
 * 
 * Encoders cannot be constructed except in this class, and it is important to
 * check your imports to make sure you are not interfering with an
 * edu.wpi.first.wpilibj.Encoder.
 * 
 * Most wpilibj encoder use cases look for velocity over 100ms, and this is
 * reflected here.
 */

public class EncoderConverter {
    // unused/unowned encoders commented out for clarity
    // public static final Encoder Grayhill25 = new Encoder(4 * 25);
    // public static final Encoder Grayhill32 = new Encoder(4 * 32);
    // public static final Encoder Grayhill50 = new Encoder(4 * 50);
    // public static final Encoder Grayhill64 = new Encoder(4 * 64);
    // public static final Encoder Grayhill128 = new Encoder(4 * 128);
    public static final EncoderConverter Grayhill256 = new EncoderConverter(4 * 256);

    public static final EncoderConverter VersaPlanetary = new EncoderConverter(4 * 1024);

    public static final EncoderConverter NeoInternal = new EncoderConverter(42 * 4);

    private static final int msPerMin = 1000 * 60; // constant unit transformation

    private final int pulses; // how many pulses are in a single rotation of the encoder

    private EncoderConverter(int pulsesPerRotation) {
        pulses = pulsesPerRotation;
    }

    /**
     * Converts RPM to the velocity expected by a CTRE pid controller
     * 
     * @param rpm - the Rotations/minute to convert
     * @return vel - the corresponding pulser per 100ms
     */
    public double RPMtoPIDVelocity(double rpm) {
        return rpm * pulses * 100 / msPerMin;
    }

    /**
     * Converts PID velocity (pulses/100ms) to RPM
     * 
     * @param vel - the pulses per 100ms
     * @return rpm
     */
    public double PIDVelocityToRPM(double vel) {
        return vel * msPerMin / (100 * pulses);
    }

    /**
     * Converts number of pulses to number of rotations
     * 
     * @param pulses - the rotation in pulses
     * @return rotations - the number of rotations
     */
    public double pulsesToRotations(double pulses) {
        return pulses / this.pulses;
    }

    /**
     * Converts rotations of the encoder into encoder pulses
     * @param rotations
     * @return pulses
     */
    public double rotationsToPulses(double rotations) {
        return rotations * pulses;
    }
}