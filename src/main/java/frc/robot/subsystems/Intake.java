package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class Intake extends SubsystemBase {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final CANEncoder rightEnc;
    private final CANEncoder leftEnc;

    private final DoubleSolenoid intakeArm;
    private final double armCounterMax = 3;
    private double armCounter = 0;

    private static final Value ExtensionDirection = Value.kReverse;
    private static final Value RetractionDirection = Value.kForward;

    private final Encoder enc = Encoder.Grayhill256;

    public Intake() {
        leftMotor = new CANSparkMax(0, MotorType.kBrushless); // change the can number
        rightMotor = new CANSparkMax(1, MotorType.kBrushless);

        // reset
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        // follow
        leftMotor.follow(rightMotor);

        // idle
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        // encoders
        leftEnc = leftMotor.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
        rightEnc = rightMotor.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));

        intakeArm = new DoubleSolenoid(Constants.K_INTAKE_SOLENOID_UP, Constants.K_INTAKE_SOLENOID_DOWN);
        register();
    }

    @Override
    public void periodic() {

    }

    /**
     * Intake
     */
    public void intake(double speed) {
        rightMotor.set(speed);
    }

    /**
     * Extends the intake
     */
    public void extendDown() {
        intakeArm.set(ExtensionDirection);
        armCounter = armCounterMax;
    }

    /**
     * Retracts the intake
     */
    public void retractUp() {
        intakeArm.set(RetractionDirection);
        armCounter = armCounterMax;
    }

}