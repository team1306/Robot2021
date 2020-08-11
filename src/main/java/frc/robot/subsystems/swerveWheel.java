package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
//import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/*import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;*/
import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class SwerveWheel extends SubsystemBase { 
    private final CANSparkMax speedMotor;
    private final CANSparkMax angleMotor;
    //private final int wheelPosition;
    private final CANEncoder angleEnc;
    private final Encoder enc = Encoder.Grayhill256;
    private final com.revrobotics.CANPIDController pidController;

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

    public void drive(double speed, double angle) {
        speedMotor.set(speed);

        double position = angleEnc.getPosition();

        //TODO this needs to be rewritten
        double input = position + angle;
        pidController.setReference(input, ControlType.kPosition);
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