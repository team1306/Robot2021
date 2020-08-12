package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class SwerveWheel extends SubsystemBase { 
    private final CANSparkMax speedMotor;
    private final CANSparkMax angleMotor;
    //private final int wheelPosition;
    private final CANEncoder angleEnc;
    private final Encoder enc = Encoder.Grayhill256;
    private final com.revrobotics.CANPIDController pidController;

    /**
     * Creates and initializes SwerveWheel object as well as a PID controller
     * @param speedMotorID motor in charge of speed
     * @param angleMotorID motor in charge of angle
     */
    public SwerveWheel(int speedMotorID, int angleMotorID) {
        //motor providing forward acceleration
        speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);
        speedMotor.setIdleMode(IdleMode.kBrake);

        //motor providing rotation on speedMotor
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        angleEnc = angleMotor.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
        
        //creating pidController for controlling angleMotor
        pidController = angleMotor.getPIDController();
        pidController.setP(Constants.KP);
        pidController.setI(Constants.KI);
        pidController.setD(Constants.KD);
    }

    /**
     * Accepts a speed and angle 
     * Assigns values to the wheel
     * Can be used for manual testing, 
     * @param speed a value between [-1,1]
     * @param angle an angle between [0, 360]
     */
    public void drive(double speed, double angle) {
        speedMotor.set(speed);

        angle = convertAngleValue(angle);

        pidController.setReference(angle, ControlType.kPosition);
    }

    /**
     * Converts swerve into an angle value and a speed value
     * Assigns angle/speed values to the wheel
     * @param swerve swerve module
     */
    public void drive(SwerveModuleState swerve) {
        double speedMPS = swerve.speedMetersPerSecond;

        // convert to rotations per second from meters per second
        double speedValueRotations = speedMPS / (2 * Math.PI * Constants.K_WHEEL_RADIUS_METERS); 
        speedMotor.set(speedValueRotations);

        //this method returns the angle of the point on the circle created by swerve
        double angleValue = swerve.angle.getDegrees();

        //converts angleValue to a position value    
        double angle = convertAngleValue(angleValue);

        pidController.setReference(angle, ControlType.kPosition);
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
}