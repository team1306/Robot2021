package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
//import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;

/*import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;*/
import frc.robot.Constants;
import frc.robot.utils.EncoderConverter;

public class DriveTrain extends SubsystemBase { 
    private final CANSparkMax FrontRight; 
    private final CANSparkMax MiddleRight;
    private final CANSparkMax BackRight;

    private final CANSparkMax FrontLeft;
    private final CANSparkMax MiddleLeft;
    private final CANSparkMax BackLeft;

    private final EncoderConverter enc = EncoderConverter.Grayhill256;

    private final CANEncoder leftEnc;
    private final CANEncoder rightEnc; 

    public DriveTrain() {
        FrontRight = new CANSparkMax(Constants.K_DRIVE_RIGHT_FRONT_ID, MotorType.kBrushless);
        MiddleRight = new CANSparkMax(Constants.K_DRIVE_RIGHT_MID_ID, MotorType.kBrushless);
        BackRight = new CANSparkMax(Constants.K_DRIVE_RIGHT_BACK_ID, MotorType.kBrushless);
;
        FrontLeft = new CANSparkMax(Constants.K_DRIVE_LEFT_FRONT_ID, MotorType.kBrushless);
        MiddleLeft = new CANSparkMax(Constants.K_DRIVE_LEFT_MID_ID, MotorType.kBrushless);
        BackLeft = new CANSparkMax(Constants.K_DRIVE_LEFT_BACK_ID, MotorType.kBrushless);

        //reset
        FrontRight.restoreFactoryDefaults();
        MiddleRight.restoreFactoryDefaults();
        BackRight.restoreFactoryDefaults();

        FrontLeft.restoreFactoryDefaults();
        MiddleLeft.restoreFactoryDefaults();
        BackLeft.restoreFactoryDefaults();

        //following front motors
        MiddleRight.follow(FrontRight);
        BackRight.follow(FrontRight);

        MiddleLeft.follow(FrontLeft);
        BackLeft.follow(FrontLeft);

        FrontRight.setIdleMode(IdleMode.kBrake);
        MiddleRight.setIdleMode(IdleMode.kBrake);
        BackRight.setIdleMode(IdleMode.kBrake);

        FrontLeft.setIdleMode(IdleMode.kBrake);
        MiddleRight.setIdleMode(IdleMode.kBrake);
        BackRight.setIdleMode(IdleMode.kBrake);

        //declaring encoders
        rightEnc = FrontRight.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
        leftEnc = FrontLeft.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
    }

    public void drive(double verticalMovement, double horizontalMovement) {
        FrontRight.set(speed);
        FrontLeft.set(speed);
    }

    // int = {1,2,3,4}
    //double = {1.01, 1.001, 1.0001}

}