package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveWheel extends SubsystemBase { 
    private final TalonFX speedMotor;
    private final TalonFX angleMotor;
    private final CANCoder angleEnc;
    private double integral;
    private double error = 0;
    private double previousError = 0;
    private double derivative; 
   
    //private final Encoder enc = Encoder.Grayhill256;

    //possibly make these wheel specific 
    private final double KP = 1;
    private final double KI = 0;
    private final double KD = 0;

    private final boolean phaseReading;

    /**
     * Creates and initializes SwerveWheel object as well as a PID controller
     * @param speedMotorID motor in charge of speed
     * @param angleMotorID motor in charge of angle
     */
    public SwerveWheel(int speedMotorID, int angleMotorID, int CANCoderID) {
        //motor providing forward acceleration
        speedMotor = new TalonFX(speedMotorID);
        speedMotor.configureFactoryDefault();
        // speedMotor.setIdleMode(IdleMode.kBrake);

        //motor providing rotation on speedMotor
        angleMotor = new TalonFX(angleMotorID);

        angleEnc = new CANCoder(CANCoderID);

        
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        
        angleMotor.configNominalOutputForward(0);
        angleMotor.configNominalOutputReverse(0);
        angleMotor.configPeakOutputForward(1);
        angleMotor.configPeakOutputForward(-1);

        angleMotor.configAllowableClosedloopError(0, 0, 0);

        //angleMotor.config_kF(0, Constants.kGains.kF, Constants.kTimeoutMs);
		angleMotor.config_kP(0, KP, 0);
		angleMotor.config_kI(0, KI, 0);
		angleMotor.config_kD(0, KD, 0);
        
        angleMotor.setInverted(Constants.DIRECTION_FORWARD);
        angleMotor.setSensorPhase(phaseReading);
    }

    /**
     * Converts swerve into an angle value and a speed value
     * Assigns angle/speed values to the wheel
     * @param swerve swerve module
     */
    public void drive(SwerveModuleState swerve) {
        double speedMPS = swerve.speedMetersPerSecond;

        // convert to rotations per second from meters per second then to rotations per millisecond 
        double speedValueRotations = speedMPS / (2 * Math.PI * Constants.K_WHEEL_RADIUS_METERS); 
        speedMotor.set(TalonFXControlMode.Velocity, ((speedValueRotations * 4096) / 1000);

        //this method returns the angle of the point on the circle created by swerve
        double angleValue = swerve.angle.getDegrees();

        //converts angleValue to a position value between [-1, 1]  
        //TODO: simplify this, use optimize function, and only consider 90 degree turns  
        double angle = convertAngleValue(takeShortestPath(angleValue));

        angleMotor.set(TalonFXControlMode.Position, angle * 4096);
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
     * Finds the shortest rotational path a wheel can take IN DEGREES
     * Accounts for negative and positive rotations that are multiples of 180
     * @param rotation in degrees
     * consider rotation direction
     * test cases with robot
     */
    public static double takeShortestPathDegrees(double degreesPath) {
        while(degreesPath > 360) {
            degreesPath -= 360;
        }

        return degreesPath;
    }

    /**
     * Converts the shortest rotational path in degrees to rotations
     */
    public double takeShortestPathRotations(double degreesPath) {
        return takeShortestPathDegrees(degreesPath) / 360;
    }
}