package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Encoder;
import frc.robot.utils.PIDSetup;

public class Shooter extends SubsystemBase {

    public final TalonSRX flywheel;
    private final Spark kicker;
    private final DoubleSolenoid hood;
    public final Encoder flywheelEnc = Encoder.Grayhill256;

    private final double sinHighA = Math.sin(Constants.K_ANGLE_SHORT_DEGREES);
    private final double cosHighA = Math.cos(Constants.K_ANGLE_SHORT_DEGREES);
    private final double sinLowA = Math.sin(Constants.K_ANGLE_LONG_DEGREES);
    private final double cosLowA = Math.cos(Constants.K_ANGLE_LONG_DEGREES);
    // Distance in feet
    private final double maxDistHigh = 10;// ft
    private final double minDistLow = 8;// ft

    private final double h = Constants.K_TARGET_HEIGHT_FT - Constants.K_SHOOTER_HEIGHT_FT;
    private final double g = 32.2;

    private final double kP = 0.4;
    private final double kI = 0.0001;
    private final double kD = 0;

    public Shooter() {
        // initalize flywheel for PID
        flywheel = new TalonSRX(Constants.K_SHOOTER_FlYWHEEL_ID);
        PIDSetup.IntializePID(flywheel, kP, kI, kD, 0, 1, FeedbackDevice.QuadEncoder, 0, 0);

        hood = new DoubleSolenoid(Constants.K_SHOOTER_HOOD_UP_SOLENOID, Constants.K_SHOOTER_HOOD_DWN_SOLENOID);
        // Intialize other motors
        kicker = new Spark(Constants.K_SHOOTER_KICKER_SPARK);

        NetworkTableInstance.getDefault().getEntry("vision/Distance");
    }

    public void spinToRPM(double rpm) {
        flywheel.set(ControlMode.Velocity, flywheelEnc.RPMtoPIDVelocity(rpm));
    }

    public void setFlywheelPercent(double percent) {
        flywheel.set(ControlMode.PercentOutput, percent);
    }

    public double getRPM() {
        return flywheelEnc.PIDVelocityToRPM(flywheel.getSelectedSensorVelocity());
    }

    public void stopFlywheel() {
        flywheel.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Sets the hood to high angle (short distance) or low (long distance)
     * 
     * @param isHigh - if the hood should be set to high
     */
    public void setHoodHigh(boolean isHigh) {
        if (isHigh) {
            hood.set(Value.kForward);
        } else {
            hood.set(Value.kReverse);
        }
    }

    public boolean isHoodHigh() {
        return hood.get().equals(Value.kForward);
    }

    public void listenPiDistChange(EntryNotification n) {
        double dist = n.value.getDouble();

        if (dist < 0) {
            this.setFlywheelPercent(0);
            return;
        }

        if (dist > maxDistHigh) {
            this.setHoodHigh(false);
        } else if (dist < minDistLow) {
            this.setHoodHigh(true);
        }
        // else persist current hood

        // calculate speed
        if (isHoodHigh()) {
            spinToRPM(HighShotRPM(dist));
        } else {
            spinToRPM(LowShotRPM(dist));
        }
    }

    private double HighShotRPM(double dist) {
        double d2 = dist * dist;
        double repTerm = cosHighA * (sinHighA * dist - h * cosHighA);
        return feetPerSecondtoRPM(Math.sqrt(2 * d2 * g * repTerm) / (2 * repTerm));
    }

    private double LowShotRPM(double dist) {
        double d2 = dist * dist;
        double repTerm = cosLowA * (sinLowA * dist - h * cosLowA);
        return feetPerSecondtoRPM(Math.sqrt(2 * d2 * g * repTerm) / (2 * repTerm));
    }

    public double feetPerSecondtoRPM(double feetPerSecond) {
        return feetPerSecond * 12 / (Math.PI * Constants.K_SHOOTER_RADIUS_INCHES);
    }

    public double RPMtoFeetPerSecond(double rpm) {
        return rpm * Math.PI * Constants.K_SHOOTER_RADIUS_INCHES / 12;
    }
}