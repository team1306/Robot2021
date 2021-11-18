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
    private double ANGLE_MOTOR_P = .25;

    // motor that controls wheel speed
    TalonFX speedMotor;

    // external enocoder the angle motor
    CANCoder angleEnc;

    //used for relaying data about the swerve module state to shuffleboard

    //offset of the wheel in degrees
    
    //used for relaying data
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

        angleMotor.setNeutralMode(NeutralMode.Brake);
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
         //speedMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond);
         angleMotor.set(ControlMode.PercentOutput, turnPercentHelper(state));//);
        
    }
    
    /**
     * 
     * @param state containing target turn position
     * @return percent output that wheel should move to each position
     */
    public double turnPercentHelper(SwerveModuleState state) {
        var id = angleMotor.getDeviceID();
        double target = state.angle.getDegrees();
        double currentPosition = angleMotor.getSelectedSensorPosition() * Constants.ENCODER_TICKS_TO_DEGREES;

        //reduces currentPosition to be a value between (-360, 360)
        currentPosition = currentPosition % 360;

        //finds the amount of degrees that the angleMotor needs to rotate to reach the position
        //after this delta should be bounded between [-180, 180]
        double delta = (currentPosition - target); 
        if(delta > 180) delta = delta - 360;
        if(delta < -180) delta = delta + 360;
        SmartDashboard.putNumber(id + ": error", delta);

        //changing delta into a percent output between [-ANGLE_MOTOR_P,ANGLE_MOTOR_P] with ANGLE_MOTOR_P as a max speed
        delta = (delta / 180) * ANGLE_MOTOR_P;
        SmartDashboard.putNumber(id + ": %out", delta);
        SmartDashboard.putNumber(id + ": current pos", currentPosition);
        return delta;
    }


    /**
     * Prints out data to Shuffleboard based on the ID of the device that is passed
     * through
     * 
     * @param ID ID of the device to collect data from
     */
    public void shuffleboard(String ID) {
    }

}