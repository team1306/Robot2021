package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.util.Units;

/*import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;*/
import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class swerveWheel extends SubsystemBase { 
    private final CANSparkMax speedMotor;
    private final CANSparkMax angleMotor;
    //private final int wheelPosition;
    private final CANEncoder angleEnc;
    private final Encoder enc = Encoder.Grayhill256;
    private final com.revrobotics.CANPIDController pidController;

    public swerveWheel(int speedMotorID, int angleMotorID) {
        //These are the motors that provide speed
        speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        angleEnc = angleMotor.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
        pidController = angleMotor.getPIDController();
        pidController.setP(Constants.KP);
        pidController.setI(Constants.KI);
        pidController.setD(Constants.KD);

        //this.wheelPosition = wheelPosition;

        speedMotor.setIdleMode(IdleMode.kBrake);
    }

    public void drive(double speed, double angle) {
        speedMotor.set(speed);

        double position = angleEnc.getPosition();

        //TODO this needs to be rewritten
        double input = position + angle;
        pidController.setReference( input, ControlType.kPosition);
        //double velocity = angleEnc.getVelocity();

        //move to swerveDrive
        // if (speedMotor.getIdleMode().equals(IdleMode.kBrake)) {
        //     //converted to pulses
        //     if(wheelPosition == 1) {
        //         //move to 45 degree position left of the y axis
        //         pidController.setReference(.375, ControlType.kPosition);
        //     } else {
        //         //move to 45 degree position right of the y axis
        //         pidController.setReference(.125, ControlType.kPosition);
        //     }
        // } else {
            //positions = rotations: [0,256]
            //angle: [-1,1]
            //how does this work
            //pidController.setReference( , ControlType.kPosition);

        //}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       

        // double setpoint = angle * (Constants.MAX_VOLTS * 0.5) + (Constants.MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
        // if (setpoint < 0) {
        //     setpoint = Constants.MAX_VOLTS + setpoint;
        // }
        // if (setpoint > Constants.MAX_VOLTS) {
        //     setpoint = setpoint - Constants.MAX_VOLTS;
        // }
        // pidController.setSetpoint(setpoint);

        //sketch
        //wheels have to start perfectly straight
        //this will work if it is close
    }

    public void resetEncoder() {
        angleEnc.setPosition(0.0);
    }

    public double getPosition() {
        return angleEnc.getPosition();
    }

    /**
     * Converts swerve into an angle value and a speed value
     * Assigns angle/speed values to the wheel
     * @param swerve swerve module
     */
    public void convert(SwerveModuleState swerve) {

        double angleValue = swerve.angle.getDegrees(); 
        double speedMPS = swerve.speedMetersPerSecond;

        // convert to rotations per second from meters per second
        double speedValueRotations = speedMPS / (2 * Math.PI * Constants.K_WHEEL_RADIUS_METERS); 
        
        this.drive(speedValueRotations, angleValue); 
    }
}