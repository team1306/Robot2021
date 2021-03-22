package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The SwerveWheel class is responsible for running the drive and turn motors
 * within the swerve module
 */
public class SwerveWheel extends SubsystemBase {
    // swerve wheel components
    private final TalonFX speedMotor; // motor responsible for movement of the wheel
    private final TalonFX angleMotor; // motor responsible for turning of the wheel
    private final CANCoder angleEnc; // encoder that records position of the wheel

    // TODO figure out whether we make PID numbers wheel-specific

    // PID loop constants
    private final double SpeedMotor_KP = .1;
    private final double SpeedMotor_KI = 0;
    private final double SpeedMotor_KD = 0;

    private final double AngleMotor_KP = .2;
    private final double AngleMotor_KI = .0001;
    private final double AngleMotor_KD = 0;

    private SwerveModuleState swerve = null;

    private final boolean phaseReading = false;

    /**
     * Creates and initializes SwerveWheel object
     * 
     * @param speedMotorID ID of the movement motor
     * @param angleMotorID ID of the rotation motor
     * @param encoderID    ID of the encoder
     */
    public SwerveWheel(int speedMotorID, int angleMotorID, int encoderID, boolean phase) {
        // initializing the encoder
        angleEnc = new CANCoder(encoderID);
        angleEnc.configFactoryDefault();
        
        // initializing speedMotor and setting its PID loop
        speedMotor = new TalonFX(speedMotorID);
        speedMotor.configFactoryDefault();

        speedMotor.config_kP(0, SpeedMotor_KP, 0);
        speedMotor.config_kI(0, SpeedMotor_KI, 0);
        speedMotor.config_kD(0, SpeedMotor_KD, 0);
        // speedMotor.setIdleMode(IdleMode.kBrake);

        // initializing angleMotor and setting its PID loop
        angleMotor = new TalonFX(angleMotorID);
        angleMotor.configFactoryDefault();

        TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

        // Use the CANCoder as the remote sensor for the primary TalonFX PID
        angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = angleEnc.getDeviceID();
        angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;

        angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        angleMotor.configAllSettings(angleTalonFXConfiguration);

        angleMotor.config_kP(0, AngleMotor_KP, 0);
        angleMotor.config_kI(0, AngleMotor_KI, 0);
        angleMotor.config_kD(0, AngleMotor_KD, 0);

        angleMotor.configNominalOutputForward(0);
        angleMotor.configNominalOutputReverse(0);
        angleMotor.configPeakOutputForward(1);
        angleMotor.configPeakOutputReverse(-1);

        //angleMotor.setNeutralMode(NeutralMode.Brake);

        angleMotor.setInverted(Constants.DIRECTION_FORWARD);
        angleMotor.setSensorPhase(phase);
        angleMotor.configFeedbackNotContinuous(false, 0);
    }

    /**
     * Calculates and assigns the vector that a swerve module is supposed to be
     * moving at. Includes the angle of the wheel and the speed that the wheel is
     * supposed to be moving at
     * 
     * @param swerve swerve module to assign values to
     */
    public void drive(SwerveModuleState swerve) {
        this.swerve = swerve;
        double speedMPS = swerve.speedMetersPerSecond;

        // convert to rotations per second from meters per second then to rotations per
        // millisecond
        double speedValueRotations = speedMPS / (2 * Math.PI * Constants.K_WHEEL_RADIUS_METERS);
        speedMotor.set(ControlMode.Velocity, ((speedValueRotations * 4096.0) / 10.0));

        Rotation2d currentRotation = Rotation2d.fromDegrees(angleEnc.getAbsolutePosition());
        swerve = optimize(swerve, currentRotation);

        double angleValue = swerve.angle.getDegrees();

        angleMotor.set(ControlMode.Position, (angleValue / 360.0) * 4096);

    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState(-desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    /**
     * Finds the shortest turn the robot can make given the degree turn that it
     * originally planned to make (a 360 degree turn is a 0 degree turn)
     * 
     * @param degreesPath angle that the robot wants to turn
     */
    public static double convertToPositiveDegrees(double degreesPath) {
        degreesPath = (degreesPath > 0) ? degreesPath : 360 - degreesPath;
        return ((degreesPath % 360) + 360) % 360;
    }

    /**
     * Converts the speed of a swerve module to PercentOutput, which is used in
     * TalonFX ControlMode. This method returns a value [-1, 1]
     * 
     * @param swerve converts the speed from this swerve module to PercentOutput
     */
    public double convertToPercentOutput(SwerveModuleState swerve) {
        double speedMPS = swerve.speedMetersPerSecond;
        return speedMPS / Constants.FASTEST_SPEED_METERS;
    }

    /**
     * Returns the PercentOutput of the speed of a swerve module
     * 
     * @param swerve speed of this swerve module
     */
    public double getSpeedDrive(SwerveModuleState swerve) {
        return convertToPercentOutput(swerve);
    }

    /**
     * Resets the position of the encoder
     */
    public void resetEncoder() {
        angleEnc.setPosition(0.0);
    }

    /**
     * Sets the position of the encoder to the specified position
     * 
     * @param position position to set encoder to
     */
    public void setEncoder(double position) {
        angleEnc.setPosition(position);
    }

    /**
     * Gets the position of the encoder
     */
    public double getPosition() {
        return angleEnc.getPosition();
    }

    /**
     * Returns angle of swerve module in degrees
     */
    public double getAngleValueDegrees() {
        return angleEnc.getAbsolutePosition();
    }

    /**
     * Prints out data to Shuffleboard based on the ID of the device that is passed
     * through
     * 
     * @param ID ID of the device to collect data from
     */
    public void shuffleboard(String ID) {
        SmartDashboard.putNumber(ID + ":Current Position",
                (angleMotor.getSelectedSensorPosition()));
        SmartDashboard.putNumber(ID + ":Target Angle Position", convertToPositiveDegrees(swerve.angle.getDegrees()));
        SmartDashboard.putNumber(ID + ":Target Motor Speed", swerve.speedMetersPerSecond);
        SmartDashboard.putNumber(ID + ":Turn motor velocity", angleMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber(ID + ":Target PID Error", angleMotor.getClosedLoopError());
        SmartDashboard.putNumber(ID + ":Target PID Target", angleMotor.getClosedLoopTarget());

    }
}