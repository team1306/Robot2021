package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This class takes in a SwerveModuleState and finds the target values: angle
 * and speed and then applies them to the wheel.
 * 
 * We have two motors that are used to control the swerve module. One controls
 * speed and the other controls angle. On the angle motor we use an encoder to
 * help for accuracy on wheel rotation.
 * 
 * This class will also provide data to UserDrive to be used for SmartDashboard.
 */
public class SwerveWheel extends SubsystemBase {

    // used for controlling wheel direction
    TalonFX angleMotor;

    // PID constants for the angleMotor
    // values are based off of 1023 being full output (i.e. kP *
    // errorThatShouldResultInMaxOutput = 1023)
    // to calculate
    double angleMotor_P = .1;
    double angleMotor_I = 0;
    double angleMotor_D = 0;

    // used for controlling wheel speed
    TalonFX speedMotor;

    // PID constants for the speedMotor
    // values are based off of 1023 being full output (i.e. kP *
    // errorThatShouldResultInMaxOutput = 1023)
    // to calculate
    double speedMotor_P = .1;
    double speedMotor_I = 0;
    double speedMotor_D = 0;

    // used for accuracy on wheel rotation
    CANCoder angleEnc;

    //used for relaying data to shuffleboard
    SwerveModuleState swerve = null;

    /**
     * Initializes motors
     * 
     * @param angleMotorID
     * @param speedMotorID
     * @param encoderID
     * @param offset       in degrees
     */
    public SwerveWheel(int angleMotorID, int speedMotorID, int encoderID, double offset) {
        // initialize and reset the encoder
        angleEnc = new CANCoder(encoderID);
        angleEnc.configFactoryDefault();

        // initialize and reset the angle motor
        angleMotor = new TalonFX(angleMotorID);
        angleMotor.configFactoryDefault();

        // Configuring the PID constants for the angle motor
        angleMotor.config_kP(0, speedMotor_P);
        angleMotor.config_kI(0, speedMotor_I);
        angleMotor.config_kD(0, speedMotor_D);

        // initialize and reset the speed motor
        speedMotor = new TalonFX(speedMotorID);
        speedMotor.configFactoryDefault();

        // Configuring the PID constants for the speed motor
        speedMotor.config_kP(0, angleMotor_P);
        speedMotor.config_kI(0, angleMotor_I);
        speedMotor.config_kD(0, angleMotor_D);

        // Configuring the offset so that all wheels 0 is at the same spot
        // Moves the wheels to that spot

        // param in degrees
        angleEnc.configMagnetOffset(offset);

        // param in encoder ticks
        angleMotor.setSelectedSensorPosition(angleEnc.getAbsolutePosition() * Constants.DEGREES_TO_ENCODER_TICKS);

        // param in encoder ticks
        angleMotor.set(ControlMode.Position, 0);

    }

    /**
     * Takes in a state. From that state it receives the target speed in meter per
     * second and the target angle in rotation 2D.
     * 
     * It uses the state to find the speed the speedMotor should go and the angle
     * the angleMotor should turn to then sets the motor to those values so the
     * robot works!
     * 
     * (combining two methods)
     * 
     * @param state
     */
    public void drive(SwerveModuleState state) {
        //storing state as a global variable to get its information
        swerve = state; 

        // gets the current angle from the angle enc and optimizes the module so it
        // doesn't do extra rotations
        Rotation2d currentAngle = Rotation2d.fromDegrees(getAngle());
        state = SwerveModuleState.optimize(state, currentAngle);

        // call setSpeed and setRotation with proper values from our SwerveModuleState
        //setSpeed(state.speedMetersPerSecond);
        //setPercent(.2);
        setTurnPercent(.1);
        //setRotation(state.angle);

       // ControlMode.PercentOutput(0.2);
    }

    // setting motor speed
    private void setSpeed(double targetSpeedMPS) {
        // convert to controller native units = encoder pulses / 100 ms
        // meters per second => meters per 100 ms => encoder pulses per 100ms
        double targetSpeedToMP100ms = (targetSpeedMPS / 10);
        double targetSpeedNativeUnits = (targetSpeedToMP100ms / Constants.K_WHEEL_CIRCUMFERENCE_METERS) * 4096.0;
        // TODO remeasure the wheels circumference

        // set motor equal to ^^
        speedMotor.set(ControlMode.Velocity, targetSpeedNativeUnits);
    }

    private void setPercent(double percentOutput) {
        speedMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    private void setTurnPercent(double percentOutput) {
        angleMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    // setting angle rotation
    private void setRotation(Rotation2d targetRotation) {
        // convert to the controller native units = encoder pulses
        double targetRotationEncoderPulses = targetRotation.getDegrees() * Constants.DEGREES_TO_ENCODER_TICKS;

        // set angle pos to ^^
        angleMotor.set(ControlMode.Position, targetRotationEncoderPulses);
    }

    /**
     * turn wheels to 0 position. used for testing
     */
    private void setRotationToZero() {
        angleMotor.set(ControlMode.Position, 0);
    }

    /**
     * @return the current degree rotation of the angle wheel
     */
    public double getAngle() {
        return angleMotor.getSelectedSensorPosition() * Constants.ENCODER_TICKS_TO_DEGREES;
    }

    /**
     * Prints out data to Shuffleboard based on the ID of the device that is passed
     * through
     * 
     * @param ID ID of the device to collect data from
     */
    public void shuffleboard(String ID) {
        // SmartDashboard.putNumber(ID + ":Current Position", angleMotor.getSelectedSensorPosition());
        // SmartDashboard.putNumber(ID + ":Target Angle Position", swerve.angle.getDegrees());
        // SmartDashboard.putNumber(ID + ":Target Motor Speed", swerve.speedMetersPerSecond);
        // SmartDashboard.putNumber(ID + ":Target PID Error", angleMotor.getClosedLoopError());
        // SmartDashboard.putNumber(ID + ":Target PID Target", angleMotor.getClosedLoopTarget());
    }
}