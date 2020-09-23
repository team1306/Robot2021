package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class Intake extends SubsystemBase {

  // Two motors for the recad robot intake
  private final CANSparkMax motorRightLeader;
  private final CANSparkMax motorLeftFollower;

  /*
   * I left this stuff here in case we want to put encoders back. Should be similar to drivetrain if we do
   * // The encoders for the two intake motors private final CANEncoder rightEnc;
   * private final CANEncoder leftEnc;
   * 
   * // Just the Encoder that egan wrote. This is here for it's methods. private
   * final Encoder enc = Encoder.Grayhill256;
   */

  // piston
  private final DoubleSolenoid intakeArm;

  // piston direction values
  private static final Value ExtensionDirection = Value.kForward; // Pretty sure extension is kForward
  private static final Value RetractionDirection = Value.kReverse; // I think this is alright using the Value thing.

  /**
   * Creates a new Intake subsystem.
   */
  public Intake() {

    // delcares the motor controllers
    motorRightLeader = new CANSparkMax(Constants.K_INTAKE_MOTOR_ID_RIGHT, MotorType.kBrushless);
    motorLeftFollower = new CANSparkMax(Constants.K_INTAKE_MOTOR_ID_LEFT, MotorType.kBrushless);

    // reset
    motorRightLeader.restoreFactoryDefaults();
    motorLeftFollower.restoreFactoryDefaults();

    // follow
    motorLeftFollower.follow(motorRightLeader);

    // idle
    motorRightLeader.setIdleMode(IdleMode.kBrake);
    motorLeftFollower.setIdleMode(IdleMode.kBrake);

    // The piston stuff
    intakeArm = new DoubleSolenoid(Constants.K_INTAKE_SOLENOID_UP, Constants.K_INTAKE_SOLENOID_DOWN);
    register();

  }

  /**
   * This method spins the motor so the power cells go in
   */
  public void spin() {
    motorRightLeader.set(0.5);
  }

  /*
   * This is just the spin() method again, except it takes a parameter. The other
   * one is there just in case you want to intake without giving it any input for
   * speed.
   */
  public void spin(double speed) {
    motorRightLeader.set(speed);
  }

  /**
   * This method spins the motor backwards to un-jam power cells
   */
  public void spit() {
    motorRightLeader.set(-0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (vision stuff)
  }

  /**
   * Retracts the intake
   */
  public void retract() {
    intakeArm.set(RetractionDirection);
  }

  /**
   * Extends the intake
   */
  public void extend() {
    intakeArm.set(ExtensionDirection);
  }

}