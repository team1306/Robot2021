package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Encoder;
import frc.robot.utils.NetworkTablePaths;

public class DriveTrain extends SubsystemBase {

  private final CANSparkMax rightLeader;
  private final CANSparkMax leftLeader;
  private final CANSparkMax rightFollower1;
  private final CANSparkMax leftFollower1;
  private final CANSparkMax rightFollower2;
  private final CANSparkMax leftFollower2;

  private final CANEncoder rightEnc;
  private final CANEncoder leftEnc;

  private final DoubleSolenoid shift;
  private final double shiftCounterMax=3;
  private double shiftCounter=0;

  public static final DoubleSolenoid.Value K_HIGH_GEAR = DoubleSolenoid.Value.kForward;
  public static final DoubleSolenoid.Value K_LOW_GEAR = DoubleSolenoid.Value.kReverse;

  private final Encoder enc = Encoder.Grayhill256;
  public final AHRS gyro;

  private NetworkTableEntry putHeading;

  public DriveTrain() {
    // initialize motor controllers
    // right
    rightLeader = new CANSparkMax(Constants.K_DRIVE_RIGHT_FRONT_ID, MotorType.kBrushless);
    rightFollower1 = new CANSparkMax(Constants.K_DRIVE_RIGHT_MIDDLE_ID, MotorType.kBrushless);
    rightFollower2 = new CANSparkMax(Constants.K_DRIVE_RIGHT_BACK_ID, MotorType.kBrushless);

    shift= new DoubleSolenoid(Constants.K_DRIVE_SHIFT_HIGH, Constants.K_DRIVE_SHIFT_LOW);

    // left
    leftLeader = new CANSparkMax(Constants.K_DRIVE_LEFT_FRONT_ID, MotorType.kBrushless);
    leftFollower1 = new CANSparkMax(Constants.K_DRIVE_LEFT_MIDDLE_ID, MotorType.kBrushless);
    leftFollower2 = new CANSparkMax(Constants.K_DRIVE_LEFT_BACK_ID, MotorType.kBrushless);

    leftLeader.restoreFactoryDefaults();
    leftFollower1.restoreFactoryDefaults();
    leftFollower2.restoreFactoryDefaults();

    rightLeader.restoreFactoryDefaults();
    rightFollower1.restoreFactoryDefaults();
    rightFollower2.restoreFactoryDefaults();
    // set follow
    rightFollower1.follow(rightLeader);
    rightFollower2.follow(rightLeader);

    leftFollower1.follow(leftLeader);
    leftFollower2.follow(leftLeader);

    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower1.setIdleMode(IdleMode.kBrake);
    rightFollower2.setIdleMode(IdleMode.kBrake);
    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower1.setIdleMode(IdleMode.kBrake);
    leftFollower2.setIdleMode(IdleMode.kBrake);

    // get encoders
    rightEnc = rightLeader.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
    leftEnc = leftLeader.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));

    gyro = new AHRS();

    putHeading = NetworkTableInstance.getDefault().getEntry(NetworkTablePaths.robotHeading);

    register();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putHeading.setDouble(this.getHeadingDegrees());
    if(shiftCounter ==0){
      shift.set(Value.kOff);
    }else{
      shiftCounter-=1;
    }
  }

  public void tankDrive(double rightOutput, double leftOutput) {
    rightLeader.set(rightOutput);
    leftLeader.set(-leftOutput);
  }

  // /**
  //  * @param rightVel - the RPM velocity for the right half of the robot
  //  * @param leftVel  - the RPM velocity for the left half of the robot
  //  */
  // public void velocityDrive(double rightVel, double leftVel) {
  //   rightLeader.getPIDController().setReference(rightVel, ControlType.kVelocity);
  //   leftLeader.getPIDController().setReference(leftVel, ControlType.kVelocity);
  // }

  public double getRightRPM() {
    return rightEnc.getVelocity();
  }

  /**
   * @return rotations
   */
  public double getRightPos() {
    return rightEnc.getPosition();
  }

  public double getLeftRPM() {
    return leftEnc.getVelocity();
  }

  /**
   * @return rotations
   */
  public double getLeftPos() {
    return leftEnc.getPosition();
  }

  public double getLeftPercentOut(){
    return leftLeader.get();
  }

  public double getRightPercentOut(){
    return leftLeader.get();
  }

  public double getHeadingDegrees() {
    return gyro.getAngle();
  }

  public double getRotVelocity() {
    return gyro.getRate();
  }

  public void resetHeading() {
    gyro.reset();
  }

  // public void setPID(double kP, double kI, double kD) {
  //   // right PID configuration
  //   rightLeader.getPIDController().setP(kP);
  //   rightLeader.getPIDController().setI(kI);
  //   rightLeader.getPIDController().setD(kD);
  //   // left PID configuration
  //   leftLeader.getPIDController().setP(kP);
  //   leftLeader.getPIDController().setI(kI);
  //   leftLeader.getPIDController().setD(kD);
  // }

  /**
   * Turns by rotations relative to current location
   */
  public void positionDrive(double rightRotations, double leftRotations) {
    double goalRight = rightRotations + getRightPos();
    double goalLeft = leftRotations + getLeftPos();
    rightLeader.getPIDController().setReference(enc.rotationsToPulses(goalRight), ControlType.kPosition);
    leftLeader.getPIDController().setReference(enc.rotationsToPulses(goalLeft), ControlType.kPosition);

  }

  /**
   * Swaps the gear of the drive train
   */
  public void shift() {
    if (shift.get().equals(K_HIGH_GEAR)) {
      shift.set(K_LOW_GEAR);
    } else {
      shift.set(K_HIGH_GEAR);
    }
    shiftCounter = shiftCounterMax;
  }

  /**
   * Shifts to given gear. Gears are determined by K_HIGH_GEAR and K_LOW_GEAR
   * 
   * @param gear
   */
  public void shift(DoubleSolenoid.Value gear) {
    shift.set(gear);
    shiftCounter = shiftCounterMax;
  }

  public boolean isShiftLow(){
    return shift.get().equals(K_LOW_GEAR);
  }

  public double metersToRotations(double meters) {
    return meters * 100 / (2.54 * Constants.K_WHEEL_RADIUS_INCHES * Math.PI);
  }

  public double feetToRotations(double feet) {
    return feet * 12 / (Constants.K_WHEEL_RADIUS_INCHES * Math.PI);
  }

  public double rotationsToFeet(double rotations) {
    return rotations * Constants.K_WHEEL_RADIUS_INCHES * Math.PI / 12;
  }
}
