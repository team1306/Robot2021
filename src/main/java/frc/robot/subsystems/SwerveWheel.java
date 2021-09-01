package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
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
    //works for slow acceleration, but it doesn't work for sudden changes
    //need to test on the ground
    double angleMotor_P = .1;
    double angleMotor_I = 0;
    double angleMotor_D = 0;

    // used for controlling wheel speed
    TalonFX speedMotor;


    // PID constants for the speedMotor
    // values are based off of 1023 being full output (i.e. kP *
    // errorThatShouldResultInMaxOutput = 1023)
    // to calculate
    // p val of 0.1 behaves as expected (although motor motion is choppy)
    double speedMotor_P = .065;//ouculations on the back left and front right motors with p value .15
    double speedMotor_I = 0;
    //.05 doesn't do anything no behavior
    double speedMotor_D = 0;

    // used for accuracy on wheel rotation
    CANCoder angleEnc;

    //used for relaying data to shuffleboard
    SwerveModuleState swerve = new SwerveModuleState();

    double targetSpeed = 0;
    /**
     * Initializes motors
     * 
     * @param angleMotorID
     * @param speedMotorID
     * @param encoderID
     * @param offset       in degrees
     */
    public SwerveWheel(int speedMotorID, int angleMotorID, int encoderID, double offset) {
        // initialize and reset the encoder
        angleEnc = new CANCoder(encoderID);
        angleEnc.configFactoryDefault();

        // initialize and reset the angle motor
        angleMotor = new TalonFX(angleMotorID);
        angleMotor.configFactoryDefault();

        // Configuring the PID constants for the angle motor
        angleMotor.config_kP(0, angleMotor_P);
        angleMotor.config_kI(0, angleMotor_I);
        angleMotor.config_kD(0, angleMotor_D);

        // initialize and reset the speed motor
        speedMotor = new TalonFX(speedMotorID);
        speedMotor.configFactoryDefault();

        // Configuring the PID constants for the speed motor
        speedMotor.config_kP(0, speedMotor_P);
        speedMotor.config_kI(0, speedMotor_I);
        speedMotor.config_kD(0, speedMotor_D);

        //SmartDashboard.putNumber("Speed Motor P-Error: ", pError.value);

        // Configuring the offset so that all wheels 0 is at the same  spot
        // Moves the wheels to that spot

        // param in degrees
        angleEnc.configMagnetOffset(offset);

        // param in encoder ticks
        angleMotor.setSelectedSensorPosition(angleEnc.getAbsolutePosition() * Constants.DEGREES_TO_ENCODER_TICKS);

        // param in encoder ticks
        angleMotor.set(ControlMode.Position, offset * Constants.DEGREES_TO_ENCODER_TICKS);

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
        // there is jittering but it doesn't sound like bad jittering
        // down stick does not work
        // WE CHANGED IT TO SPEED MOTOR AND NOT ANGLE MOTOR !!
        // problem before was zero movement ON SPEED motor and it didnt change !

        // call setSpeed and setRotation with proper values from our SwerveModuleState
        setSpeed(state.speedMetersPerSecond);

        //setPercent(state.speedMetersPerSecond / 5.0);
        // NOTES: ^up stick and up diagonals did NOT work when using PercentOutput
    }


    /**
     * Sets the speed of the drive motor to be targetSpeedMPS (in meters per second)
     * @param targetSpeedMPS
     */
    private void setSpeed(double targetSpeedMPS) {
        targetSpeed = targetSpeedMPS;
        // convert to controller native units = encoder pulses / 100 ms
        // meters per second => meters per 100 ms => encoder pulses per 100ms
        double targetSpeedToMP100ms = (targetSpeedMPS / 10);
        double targetSpeedNativeUnits = (targetSpeedToMP100ms / Constants.K_WHEEL_CIRCUMFERENCE_METERS) * 4096.0;
        // TODO remeasure the wheels circumference
    
        // set motor equal to ^^
        speedMotor.set(ControlMode.Velocity, targetSpeedNativeUnits);
    }

    // TODO values are coming out of speedMetersPerSecond and angle, need to get ControlMode.Postion working
    // OBSERVED BEHAVIOR: wheel oscillates BackRightVoltageOutput positive neg ative
    // potential expected behavior due to the wheel being not on the ground, friction could improve
    


    // setting angle rotation
    private void setRotation(Rotation2d targetRotation) {
        // convert to the controller native units = encoder pulses
        double targetRotationEncoderPulses = targetRotation.getDegrees() * Constants.DEGREES_TO_ENCODER_TICKS;

        // set angle pos to ^^
        angleMotor.set(ControlMode.Position, targetRotationEncoderPulses);
    }

    /**
     * Sets the percent speed of the drive motor to percentOutput
     * @param percentOutput
     */
    private void setPercent(double percentOutput) {
        speedMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    /**
     * Sets the percent speed of the angle motor to percentOutput
     * @param percentOutput
     */
    private void setTurnPercent(double percentOutput) {
        angleMotor.set(ControlMode.PercentOutput, percentOutput);
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
        //SmartDashboard.putNumber(ID + ":Current Position", angleMotor.getSelectedSensorPosition());
        //SmartDashboard.putNumber(ID + ":Target Angle Position", swerve.angle.getDegrees());
        //SmartDashboard.putNumber(ID + ":Target Motor Speed", swerve.speedMetersPerSecond);
        //SmartDashboard.putNumber(ID + ":Target PID Error", angleMotor.getClosedLoopError());
        //SmartDashboard.putNumber(ID + ":Target PID Target", angleMotor.getClosedLoopTarget());
        SmartDashboard.putNumber(ID + "Voltage Output", speedMotor.getMotorOutputPercent());
        SmartDashboard.putNumber(ID + "targetSpeedMPS:", targetSpeed);

    }
}