package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveWheel extends SubsystemBase { 
    //Fields for motors in swerve wheel
    private final TalonFX speedMotor;
    private final TalonFX angleMotor;
    private final CANCoder angleEnc;

    //private final Encoder enc = Encoder.Grayhill256;

    //possibly make these wheel specific 
    //constants for PID loop
    private final double KP = .05;
    private final double KI = 0;
    private final double KD = 0;

    private SwerveModuleState swerve = null;

    private final boolean phaseReading = true;

    /**
     * Creates and initializes SwerveWheel object as well as a PID controller
     * @param speedMotorID motor in charge of speed
     * @param angleMotorID motor in charge of angle
     */
    public SwerveWheel(int speedMotorID, int angleMotorID, int CANCoderID) {
        //motor providing forward acceleration
        speedMotor = new TalonFX(speedMotorID);
        speedMotor.configFactoryDefault();

        speedMotor.config_kP(0, KP, 0);
		speedMotor.config_kI(0, KI, 0);
		speedMotor.config_kD(0, KD, 0);
        // speedMotor.setIdleMode(IdleMode.kBrake);

        //motor providing rotation on speedMotor
        angleMotor = new TalonFX(angleMotorID);
        angleMotor.configFactoryDefault();

        
        angleEnc = new CANCoder(CANCoderID);

        angleEnc.configFactoryDefault();

        TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

        // Use the CANCoder as the remote sensor for the primary TalonFX PID
        angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = angleEnc.getDeviceID();
        angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    
        angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        angleMotor.configAllSettings(angleTalonFXConfiguration);

        // angleMotor.configNominalOutputForward(0);
        // angleMotor.configNominalOutputReverse(0);
        // angleMotor.configPeakOutputForward(1);
        // angleMotor.configPeakOutputForward(-1);

        // angleMotor.configAllowableClosedloopError(0, 0, 0);

        angleMotor.setNeutralMode(NeutralMode.Brake);

		angleMotor.config_kP(0, KP, 0);
		angleMotor.config_kI(0, KI, 0);
		angleMotor.config_kD(0, KD, 0);
        
        // angleMotor.setInverted(Constants.DIRECTION_FORWARD);
        // angleMotor.setSensorPhase(phaseReading);
    }

    /**
     * Converts swerve into an angle value and a speed value
     * Assigns angle/speed values to the wheel
     * @param swerve swerve module
     */
    public void drive(SwerveModuleState swerve) {
        this.swerve = swerve;
        double speedMPS = swerve.speedMetersPerSecond;

        // convert to rotations per second from meters per second then to rotations per millisecond 
        double speedValueRotations = speedMPS / (2 * Math.PI * Constants.K_WHEEL_RADIUS_METERS); 
        speedMotor.set(TalonFXControlMode.Velocity, ((speedValueRotations * 4096) / 10));

        //this method returns the angle of the point on the circle created by swerve
        //returns [-180, 180]
        double angleValue = swerve.angle.getDegrees();

        angleValue = convertToPositiveDegrees(angleValue);

        //converts angleValue to a position value between [-1, 1]  
        //TODO: simplify this, use optimize function, and only consider 90 degree turns  
        angleMotor.set(TalonFXControlMode.Position,  (angleValue / 360) * 4096);
    }

    /**
     * returns angle of swerve module in degrees
     */
    public double getAngleValueDegrees() {
        return angleEnc.getAbsolutePosition();
    } 

    /**
     * returns percent output of speed of the swerve module
     * @param swerve
     * @return
     */
    public double getSpeedDrive(SwerveModuleState swerve) {
        return convertToPercentOutput(swerve);
    }
 
    

    /**converts a value in degrees into a value between -1 and 1
    * 0 is the point (1,0)
    * 1 is the point (-1,0), values with a positive y coordinate are considered positive
    * -1 is the point (-1,0), values with a negative y coordinate are considered negitive
    * this method should be tested with the robot to see if the robot drives as expected
    * @param angle a value between [0,360]
    */
    public double convertAngleValue(double angle) {
        if(angle < 180) {
            angle = angle / 180;
        } else if(angle >= 180) {
            angle = (angle - 360) / 180;
        }
        return angle;
    }

    public void resetEncoder() {
        angleEnc.setPosition(0.0);
    }

    public void resetEncoder(double position) {
        angleEnc.setPosition(position);
    }

    public double getPosition() {
        return angleEnc.getPosition();
    }

    /**
     * finds the shortest turn
     * @param rotation in degrees
     * consider rotation direction
     * test cases with robot
     */
    public static double convertToPositiveDegrees(double degreesPath) {
        degreesPath = (degreesPath > 0) ? degreesPath : 360 - degreesPath;
        return ((degreesPath % 360) + 360) % 360;
    }

    /**
     * thismetho dconverts to perent output
     * returns a value between [-1, 1]
     */
    public double convertToPercentOutput(SwerveModuleState swerve) {
        double speedMPS = swerve.speedMetersPerSecond;
        return speedMPS / Constants.FASTEST_SPEED_METERS;
    }

    public void smartDashboardInfo(String ID) {
        SmartDashboard.putNumber(ID + ":Current Position", angleEnc.getAbsolutePosition());
        SmartDashboard.putNumber(ID + ":Target Angle Position", convertToPositiveDegrees(swerve.angle.getDegrees()));
        SmartDashboard.putNumber(ID + ":Target Motor Speed", swerve.speedMetersPerSecond);
    }
}