package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    public CANSparkMax flywheel;
    private final CANEncoder encoder;
    public final CANPIDController controller;
    private final TalonSRX kicker;

    private final DoubleSolenoid hood;
    private final double hoodCounterMax=3;
    private double hoodCounter=0;

    private final double sinHighA = Math.sin(Constants.K_ANGLE_SHORT_DEGREES);
    private final double cosHighA = Math.cos(Constants.K_ANGLE_SHORT_DEGREES);
    private final double sinLowA = Math.sin(Constants.K_ANGLE_LONG_DEGREES);
    private final double cosLowA = Math.cos(Constants.K_ANGLE_LONG_DEGREES);
    // Distance in feet
    private final double maxDistHigh = 10;// ft
    private final double minDistLow = 8;// ft

    private final double h = Constants.K_TARGET_HEIGHT_FT - Constants.K_SHOOTER_HEIGHT_FT;
    private final double g = 32.2;

    private final double kP = 0.0003;
    private final double kI = 0.000001;
    private final double kD = 0.000008;

    private final Value hoodUp = Value.kReverse;
    private final Value hoodDown = Value.kForward;

    public Shooter() {
        // initalize flywheel for PID
        flywheel = new CANSparkMax(Constants.K_SHOOTER_FlYWHEEL_ID, MotorType.kBrushless);
        encoder = flywheel.getEncoder();
        controller = flywheel.getPIDController();
        hood = new DoubleSolenoid(Constants.K_SHOOTER_HOOD_UP_SOLENOID, Constants.K_SHOOTER_HOOD_DWN_SOLENOID);
        // Intialize other motors
        kicker = new TalonSRX(Constants.K_SHOOTER_KICKER_ID);
        kicker.setInverted(true);

        // Initialize PID
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setOutputRange(0, 1);
        controller.setFeedbackDevice(encoder);
        flywheel.setIdleMode(IdleMode.kCoast);

        register();
    }

    @Override
    public void periodic() {
        super.periodic();
        if(hoodCounter ==0){
            hood.set(Value.kOff);
        }else{
            hoodCounter-=1;
        }
    }

    public void spinToRPM(final double rpm) {
       controller.setReference(rpm, ControlType.kVelocity);
    }
    
    public void setFlywheelPercent(final double percent) {
        controller.setReference(percent, ControlType.kDutyCycle);
        //flywheel.set(percent);
    }

    public void setKickerPercent(final double percent) {
        kicker.set(ControlMode.PercentOutput, percent);
    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    /**
     * Cuts all output to the flywheel
     */
    public void stopFlywheel() {
        flywheel.stopMotor();
    }

    /**
     * Sets the hood to high angle (short distance) or low (long distance)
     * 
     * @param isHigh - if the hood should be set to high
     */
    public void setHood(final boolean isHigh) {
        if (isHigh) {
            hood.set(hoodUp);
        } else {
            hood.set(hoodDown);
        }
        hoodCounter = hoodCounterMax;
    }

    /**
     * @return if the current hood output is high
     */
    public boolean isHoodUp() {
        return hood.get().equals(hoodUp);
    }

    /**
     * Gets the rpm for a high-angle shot at given distance
     * 
     * @param dist - feet
     * @return rpm
     */
    private double HighShotRPM(final double dist) {
        final double d2 = dist * dist;
        final double repTerm = cosHighA * (sinHighA * dist - h * cosHighA);
        return feetPerSecondtoRPM(Math.sqrt(2 * d2 * g * repTerm) / (2 * repTerm));
    }

    /**
     * Gets the rpm for a low-angle shot at given distance
     * 
     * @param dist- feet
     * @return rpm
     */
    private double LowShotRPM(final double dist) {
        final double d2 = dist * dist;
        final double repTerm = cosLowA * (sinLowA * dist - h * cosLowA);
        return feetPerSecondtoRPM(Math.sqrt(2 * d2 * g * repTerm) / (2 * repTerm));
    }

    /**
     * Calculates the rpm required to launch the ball at a given speed
     * 
     * @param feetPerSecond
     * @return rpm
     */
    public double feetPerSecondtoRPM(final double feetPerSecond) {
        return feetPerSecond * 12 / (Math.PI * Constants.K_SHOOTER_RADIUS_INCHES);
    }

    /**
     * Calculates the speed a ball would be launched at a given rpm
     * 
     * @param rpm
     * @return speed - feet per second
     */
    public double RPMtoFeetPerSecond(final double rpm) {
        return rpm * Math.PI * Constants.K_SHOOTER_RADIUS_INCHES / 12;
    }

    /**
     * Calculates the hood and shooter inputs to shoot a specific distance
     * 
     * @param dist
     * 
     * @return new rpm setpoint
     */
    public double targetDistance(double dist) {
        if (dist < 0) {
            this.spinToRPM(0);
            return 0;
        }
        if (dist > maxDistHigh) {
            this.setHood(false);
        } else if (dist < minDistLow) {
            this.setHood(true);
        }
        // else if between ranges persist current hood

        // calculate speed
        if (isHoodUp()) {
            double rpm = HighShotRPM(dist);
            spinToRPM(rpm);
            return rpm;
        } else {
            double rpm = LowShotRPM(dist);
            spinToRPM(rpm);
            return rpm;
        }
    }
}