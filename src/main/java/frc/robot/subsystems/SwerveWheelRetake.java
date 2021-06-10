package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
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
public class SwerveWheelRetake extends SubsystemBase {

    // used for controlling wheel direction
    TalonFX angleMotor;

    // PID constants for the angleMotor
    // values are based off of 1023 being full output (i.e. kP *
    // errorThatShouldResultInMaxOutput = 1023)
    // TODO: don't start the robot with these values, can use the previous equation
    // to calculate
    double angleMotor_P = 1.0;
    double angleMotor_I = 1.0;
    double angleMotor_D = 1.0;

    // used for controlling wheel speed
    TalonFX speedMotor;

    // PID constants for the speedMotor
    // values are based off of 1023 being full output (i.e. kP *
    // errorThatShouldResultInMaxOutput = 1023)
    // TODO: don't start the robot with these values, can use the previous equation
    // to calculate
    double speedMotor_P = 1.0;
    double speedMotor_I = 1.0;
    double speedMotor_D = 1.0;

    // used for accuracy on wheel rotation
    CANCoder angleEnc;

    /**
     * Initializes motors
     * 
     * @param angleMotorID
     * @param speedMotorID
     * @param encoderID
     */
    public SwerveWheelRetake(int angleMotorID, int speedMotorID, int encoderID) {
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

        // initialize and reset the encoder
        angleEnc = new CANCoder(encoderID);
        angleEnc.configFactoryDefault();
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
        // gets the current angle from the angle enc and optimizes the module so it
        // doesn't do extra rotations
        Rotation2d currentAngle = Rotation2d.fromDegrees(getAngle());
        state = SwerveModuleState.optimize(state, currentAngle);

        // call setSpeed and setRotation with proper values from our SwerveModuleState
        setSpeed(state.speedMetersPerSecond);
        setRotation(state.angle);
    }

    // setting motor speed
    private void setSpeed(double targetSpeedMPS) {
        // convert to controller native units = encoder pulses / 100 ms
        // meters per second => meters per 100 ms => encoder pulses per 100ms
        // TODO: if we use selectedFeedbackCoefficient in initalization not sure if this
        // is necessary
        double targetSpeedToMP100ms = (targetSpeedMPS / 10);
        double targetSpeedNativeUnits = (targetSpeedToMP100ms / Constants.K_WHEEL_CIRCUMFERENCE_METERS) * 4096.0;

        // set motor equal to ^^
        speedMotor.set(ControlMode.Velocity, targetSpeedNativeUnits);
    }

    // setting angle rotation
    private void setRotation(Rotation2d targetRotation) {
        // convert to the controller native units = encoder pulses
        // TODO: if we use selectedFeedbackCoefficient in initalization not sure if this
        // is necessary
        double targetRotationEncoderPulses = (targetRotation.getDegrees() / 360) * 4096;

        // set angle pos to ^^
        angleMotor.set(ControlMode.Position, targetRotationEncoderPulses);
    }

    /**
     * @return the current angle of the angle wheel
     */
    public double getAngle() {
        return angleEnc.getPosition();
    }

    /**
     * Prints out data to Shuffleboard based on the ID of the device that is passed
     * through
     * 
     * @param ID ID of the device to collect data from
     */
    public void shuffleboard(String ID) {
        // SmartDashboard.putNumber(ID + "Position", angleEnc.getPosition());
        // SmartDashboard.putNumber(ID + "Absolute Position",
        // angleEnc.getAbsolutePosition());
        // SmartDashboard.putNumber(ID + ":Current Position",
        // (angleMotor.getSelectedSensorPosition()));
        // SmartDashboard.putNumber(ID + ":Target Angle Position",
        // convertToPositiveDegrees(swerve.angle.getDegrees()));
        // SmartDashboard.putNumber(ID + ":Target Motor Speed",
        // swerve.speedMetersPerSecond);
        // SmartDashboard.putNumber(ID + ":Turn motor velocity",
        // angleMotor.getSelectedSensorVelocity());
        // SmartDashboard.putNumber(ID + ":Target PID Error",
        // angleMotor.getClosedLoopError());
        // SmartDashboard.putNumber(ID + ":Target PID Target",
        // angleMotor.getClosedLoopTarget());
    }
}