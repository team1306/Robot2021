package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    //doesnt look to be a P value problem
    double angleMotor_P = .015;
    double angleMotor_I = .00005;
    double angleMotor_D = 0.0;

    // used for controlling wheel speed
    TalonFX speedMotor;


    // PID constants for the speedMotor
    // values are based off of 1023 being full output (i.e. kP *
    // errorThatShouldResultInMaxOutput = 1023)
    // to calculate
    // p val of 0.1 behaves as expected (although motor motion is choppy)
    double speedMotor_P = .015;
    double speedMotor_I = 0;
    double speedMotor_D = 0.075;

    // used for accuracy on wheel rotation
    CANCoder angleEnc;

    //used for relaying data to shuffleboard
    SwerveModuleState swerve = new SwerveModuleState();
    double wheelOffset;
    double targetSpeed = 0;
    /**
     * Initializes motors
     * 
     * @param angleMotorID
     * @param speedMotorID
     * @param encoderID
     * @param offset       in degrees
     */
    public SwerveWheel(int speedMotorID, int angleMotorID, int encoderID, boolean isRev, double offset, boolean isAngleReversed) {
        wheelOffset = offset;
        // initialize and reset the encoder
        angleEnc = new CANCoder(encoderID);
        angleEnc.configFactoryDefault();

        // initialize and reset the angle motor
        angleMotor = new TalonFX(angleMotorID);
        angleMotor.configFactoryDefault();
        angleMotor.setNeutralMode(NeutralMode.Coast);
        angleMotor.setInverted(isAngleReversed);

        // Configuring the PID constants for the angle motor
        angleMotor.config_kP(0, angleMotor_P);
        angleMotor.config_kI(0, angleMotor_I);
        angleMotor.config_kD(0, angleMotor_D);

        angleMotor.configFeedbackNotContinuous(false, 0);
        
        // initialize and reset the speed motor
        speedMotor = new TalonFX(speedMotorID);
        speedMotor.configFactoryDefault();
        speedMotor.setNeutralMode(NeutralMode.Coast);


        // Configuring the PID constants for the speed motor
        speedMotor.config_kP(0, speedMotor_P);
        speedMotor.config_kI(0, speedMotor_I);
        speedMotor.config_kD(0, speedMotor_D);

        speedMotor.setInverted(isRev);
        //SmartDashboard.putNumber("Speed Motor P-Error: ", pError.value);

        // param in encoder ticks
        //angleMotor.set(ControlMode.Position, offset * Constants.DEGREES_TO_ENCODER_TICKS);

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
        state = optimize(state, currentAngle);
         
        


        // call setSpeed and setRotation with proper values from our SwerveModuleState
        setSpeed(state.speedMetersPerSecond);
        setPIDTarget(state.angle.getDegrees() * Constants.DEGREES_TO_ENCODER_TICKS);

        
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }        
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
      }
    
      /**
         * @param scopeReference Current Angle
         * @param newAngle Target Angle
         * @return Closest angle within scope
         */
        private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
          double lowerBound;
          double upperBound;
          double lowerOffset = scopeReference % 360;
          if (lowerOffset >= 0) {
              lowerBound = scopeReference - lowerOffset;
              upperBound = scopeReference + (360 - lowerOffset);
          } else {
              upperBound = scopeReference - lowerOffset;
              lowerBound = scopeReference - (360 + lowerOffset);
          }
          while (newAngle < lowerBound) {
              newAngle += 360;
          }
          while (newAngle > upperBound) {
              newAngle -= 360;
          }
          if (newAngle - scopeReference > 180) {
              newAngle -= 360;
          } else if (newAngle - scopeReference < -180) {
              newAngle += 360;
          }
          return newAngle;
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
        angleMotor.set(ControlMode.Position, targetRotationEncoderPulses * Constants.GEAR_RATIO);
    }

    // setting PID target
    private void setPIDTarget(double encoderPulses) {
        angleMotor.set(ControlMode.Position, encoderPulses * Constants.GEAR_RATIO);
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
        return angleMotor.getSelectedSensorPosition() * Constants.ENCODER_TICKS_TO_DEGREES / 12.8;
    }
    
    public void resetAbsoluteZero() {
        double offsetTarget = angleEnc.getAbsolutePosition() + wheelOffset;
        if(offsetTarget < 0) {
            offsetTarget = 360 + offsetTarget;
        }
        angleMotor.setSelectedSensorPosition((offsetTarget) * Constants.GEAR_RATIO * Constants.DEGREES_TO_ENCODER_TICKS);
    }

    /**
     * Prints out data to Shuffleboard based on the ID of the device that is passed
     * through
     * 
     * @param ID ID of the device to collect data from
     */
    public void shuffleboard(String ID) {
        //SmartDashboard.putNumber(ID + ":Current Position", angleMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber(ID + ":Target Angle Position", swerve.angle.getDegrees());
        //SmartDashboard.putNumber(ID + ":Target Motor Speed", swerve.speedMetersPerSecond);

        //[-1024, 1024] expected
        SmartDashboard.putNumber(ID + ":Target PID Error", ((angleMotor.getClosedLoopError() / Constants.GEAR_RATIO) % 2048));

        //Outputing the PID target [0, 2048] expected
        SmartDashboard.putNumber(ID + ":Target PID Target", ((angleMotor.getClosedLoopTarget() / Constants.GEAR_RATIO) % 2048));
        SmartDashboard.putNumber(ID + "Voltage Output",         speedMotor.getMotorOutputVoltage());
       
        SmartDashboard.putNumber(ID + "targetSpeedMPS:", targetSpeed);
        //SmartDashboard.putBoolean(ID + "getting optimized?", isChanged);
        SmartDashboard.putNumber(ID + "targetPosition", swerve.angle.getDegrees() * Constants.DEGREES_TO_ENCODER_TICKS);
        SmartDashboard.putNumber(ID + "currentPosition", angleMotor.getSelectedSensorPosition() / Constants.GEAR_RATIO);
        SmartDashboard.putNumber(ID + "absolutePosition", angleEnc.getAbsolutePosition() + wheelOffset);
        SmartDashboard.putNumber(ID + "angle voltage output",         angleMotor.getMotorOutputVoltage());
      

    }
}