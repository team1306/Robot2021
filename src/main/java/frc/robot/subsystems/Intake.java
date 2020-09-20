package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class Intake extends SubsystemBase {

  // Two motors for the recad robot intake
  private final CANSparkMax motorRight;
  private final CANSparkMax motorLeft;

  private final CANEncoder rightEnc;
  private final CANEncoder leftEnc;

  private final Encoder enc = Encoder.Grayhill256;

  // TODO pistons on recad #1 - digital input because they go up and down

  /**
   * Creates a new Intake subsystem.
   */
  public Intake() {
    // delcares the motor controllers
    motorRight = new CANSparkMax(Constants.K_INTAKE_MOTOR_ID_RIGHT, MotorType.kBrushless);
    motorLeft = new CANSparkMax(Constants.K_INTAKE_MOTOR_ID_LEFT, MotorType.kBrushless);

    // reset
    motorRight.restoreFactoryDefaults();
    motorLeft.restoreFactoryDefaults();

    // follow
    motorRight.follow(motorLeft);

    // idle
    motorRight.setIdleMode(IdleMode.kBrake);
    motorLeft.setIdleMode(IdleMode.kBrake);

    // encoders
    rightEnc = motorRight.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
    leftEnc = motorLeft.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
  }

  /**
   * This method spins the motor so the power cells go in
   */
  public void spin() {
    motorRight.set(0.5);
  }

  /*
   * This is just the spin() method again, except it takes a parameter. The other
   * one is there just in case you want to intake without giving it any input for
   * speed.
   */
  public void spin(double speed) {
    motorRight.set(speed);
  }

  /**
   * This method spins the motor backwards to un-jam power cells
   */
  public void spit() {
    motorRight.set(-0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (vision stuff)
  }

  public double getRPM() {
    return enc.getVelocity();
  }

}