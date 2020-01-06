package frc.robot.utils;

public class Encoder {

    public static final Encoder Grayhill25 = new Encoder(4*25);
    public static final Encoder Grayhill32 = new Encoder(4*32);
    public static final Encoder Grayhill50 = new Encoder(4*50);
    public static final Encoder Grayhill64 = new Encoder(4*64);
    public static final Encoder Grayhill128 = new Encoder(4*128);
    public static final Encoder Grayhill256 = new Encoder(4*256);



    /*
     * Reference for what position and velocities pid is looking for: In Velocity
     * mode, output value is in position change / 100ms. In Position mode, output
     * value is in encoder ticks
     */
    private static final int msPerMin = 1000 * 60;

    private final int pulses;

    private Encoder(int pulsesPerRotation) {
        pulses = pulsesPerRotation;
    }

    /**
     * Converts RPM to the velocity expected by a CTRE pid controller
     * 
     * @param rpm - the Rotations/minute to convert
     * @return vel - the corresponding pulser per 100ms
     */
    public double RPMtoPIDVelocity(double rpm) {
        return pulses * rpm * 600.0 / msPerMin;
    }

    /**
     * Converts PID velocity (pulses/100ms) to RPM
     * 
     * @param vel - the pulses per 100ms
     * @return rpm
     */
    public double PIDVelocityToRPM(double vel) {
        return pulses * vel * msPerMin / 100;
    }

    public double pulsesToRotations(double pulses) {
        return pulses / this.pulses;
    }

    public double rotationsToPulses(double rotations) {
        return rotations * pulses;
    }
}