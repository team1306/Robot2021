package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.motorcontrol.*;

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
    // motor that controls the angle position
    TalonFX angleMotor;

    // PID constants for the angleMotor
    private static final double angleMotor_P = .025;
    private static final double angleMotor_I = .0001;
    private static final double angleMotor_D = 0.0000;

    // motor that controls wheel speed
    TalonFX speedMotor;

    // PID constants for the speedMotor
    private static final double speedMotor_P = .05;
    private static final double speedMotor_I = 0;
    private static final double speedMotor_D = 0.075;

    // TODO: be used in offset
    CANCoder angleEnc;

    //used for relaying data about the swerve module state to shuffleboard
    private SwerveModuleState swerve = new SwerveModuleState();

    //offset of the wheel in degrees
    private double wheelOffset;
    
    //used for relaying data
    double targetSpeed = 0;
    private double targetSpeedVal;
    /**
     * Creates a new SwerveWheel with the given speedMotor and angleMotor
     * @param speedMotorID the ID of the motor assigned in phoenix tuner
     * @param angleMotorID the ID of the motor assigned in phoenix tuner
     * @param encoderID the ID of the encoder assigned in phoenix tuner
     * @param isRev reverses the speedMotor
     * @param offset the difference between natural 0 and target 0 for the angleMotor
     * @param isAngleReversed reverses the angleMotor
     */
    public SwerveWheel(
        int speedMotorID,
        int angleMotorID,
        int encoderID
    ) {
        angleMotor = new TalonFX(angleMotorID);
        speedMotor = new TalonFX(speedMotorID);
        angleEnc = new CANCoder(encoderID);

        
    }

    /**
     * Takes in a state. From that state it receives the target speed in meter per
     * second and the target angle in rotation 2D.
     * 
     * It uses the state to find the speed the speedMotor should go and the angle
     * the angleMotor should turn to then sets the motor to those values so the
     * robot works!
     * 
     * @param state
     */
    public void drive(SwerveModuleState state) {
        speedMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond);
        angleMotor.set(
            ControlMode.PercentOutput,
            (angleMotor.getSelectedSensorPosition()*Constants.SENSOR_UNIT_TO_DEG - state.angle.getDegrees()) 
            * angleMotor_P
        );
    }

    public void setZeroPosition(double offset) {
        angleEnc.configMagnetOffset(offset);
        angleMotor.setSelectedSensorPosition((angleEnc.getAbsolutePosition() + offset) * Constants.GEAR_RATIO * Constants.DEGREES_TO_ENCODER_TICKS);
        // //angleMotor.set(ControlMode.Position,0);
    }

    /**
     * Alters a swerveModule state so the the state rotates as little as possible
     * @param desiredState the current state of the robot
     * @param currentAngle the current angle
     * @return an state that will take the most direct path to the target angle
     */
    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
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
     * @param targetSpeedMPS the target speed of the motor in MPS
     */
    private void setSpeed(double targetSpeedMPS) {
        // convert to controller native units = encoder pulses / 100 ms
        // meters per second => meters per 100 ms => encoder pulses per 100ms
        double targetSpeedToMP100ms = (targetSpeedMPS / 10);
        double targetSpeedNativeUnits = (targetSpeedToMP100ms / Constants.K_WHEEL_CIRCUMFERENCE_METERS) * 2048 * Constants.GEAR_RATIO;// * 4096.0;
        targetSpeed = targetSpeedNativeUnits;
        // set motor equal to ^^
        speedMotor.set(ControlMode.Velocity, targetSpeedNativeUnits);
    }

    /**
     * sets the rotational position of the angle motor
     * @param encoderPulses target rotational position in encoder pulses
     */
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
    private void setTurnPercent(double encoderTicks) {
        double percentError = (getAngle() - encoderTicks) / 1048.0; 
        angleMotor.set(ControlMode.PercentOutput, percentError);
    }

    /**
     * @return the current degree rotation of the angle wheel
     */
    private double getAngle() {
        return angleMotor.getSelectedSensorPosition() * Constants.ENCODER_TICKS_TO_DEGREES / Constants.GEAR_RATIO;
    }
    
    /**
     * TODO
     */
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
        // SmartDashboard.putNumber(ID + ":Target Angle Position", swerve.angle.getDegrees());
        // //SmartDashboard.putNumber(ID + ":Target Motor Speed", swerve.speedMetersPerSecond);

        // //[-1024, 1024] expected
        //SmartDashboard.putNumber(ID + ":Target PID Error", angleMotor.getClosedLoopError());

        // //Outputing the PID target [0, 2048] expected
        // SmartDashboard.putNumber(ID + ":Target PID Target", ((angleMotor.getClosedLoopTarget() / Constants.GEAR_RATIO) % 2048));
        // SmartDashboard.putNumber(ID + "Voltage Output",         speedMotor.getMotorOutputVoltage());
       
        //SmartDashboard.putNumber(ID + "targetSpeedValue:", targetSpeedVal);
        //SmartDashboard.putBoolean(ID + "getting optimized?", isChanged);
        //SmartDashboard.putNumber(ID + "targetPosition", swerve.angle.getDegrees() * Constants.DEGREES_TO_ENCODER_TICKS);
        SmartDashboard.putNumber(ID + "currentPosition", angleMotor.getSelectedSensorPosition()); // / Constants.GEAR_RATIO * Constants.ENCODER_TICKS_TO_DEGREES
        //SmartDashboard.putBoolean(ID + "current Position + Target PID Error = target Position ", Math.abs(angleMotor.getSelectedSensorPosition() + angleMotor.getClosedLoopError()) < Math.abs(swerve.angle.getDegrees() * Constants.DEGREES_TO_ENCODER_TICKS + 100));
        SmartDashboard.putNumber(ID + "absolutePosition", angleEnc.getAbsolutePosition());
        //SmartDashboard.putNumber(ID + "angle voltage output",         angleMotor.getMotorOutputVoltage());
        //SmartDashboard.putNumber(ID + "voltage output", speedMotor.getMotorOutputVoltage());
    
    }
}