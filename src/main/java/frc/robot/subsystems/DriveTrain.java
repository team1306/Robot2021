/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Encoder;
import frc.robot.utils.PIDSetup;

public class DriveTrain extends SubsystemBase {

  private final CANSparkMax rightLeader;
  private final CANSparkMax leftLeader;
  private final CANEncoder rightEnc;
  private final CANEncoder leftEnc;

  private final Encoder enc = Encoder.Grayhill256;
  public final AHRS gyro;

  // PID velocity constants
  private static final double kVP = 0.3;
  private static final double kVI = 0.01;
  private static final double kVD = 0;

  public DriveTrain() {
    // initialize motor controllers
    // right
    rightLeader = new CANSparkMax(Constants.K_DRIVE_RIGHT_FRONT_ID, MotorType.kBrushless);
    CANSparkMax rightFollower1 = new CANSparkMax(Constants.K_DRIVE_RIGHT_BACK_ID, MotorType.kBrushless);
    CANSparkMax rightFollower2 = new CANSparkMax(Constants.K_DRIVE_RIGHT_BACK_ID, MotorType.kBrushless);
    // left
    leftLeader = new CANSparkMax(Constants.K_DRIVE_LEFT_FRONT_ID, MotorType.kBrushless);
    CANSparkMax leftFollower1 = new CANSparkMax(Constants.K_DRIVE_LEFT_BACK_ID, MotorType.kBrushless);
    CANSparkMax leftFollower2 = new CANSparkMax(Constants.K_DRIVE_LEFT_BACK_ID, MotorType.kBrushless);
    // set follow
    rightFollower1.follow(rightLeader);
    rightFollower2.follow(rightLeader);

    leftFollower1.follow(leftLeader);
    leftFollower2.follow(leftLeader);
    // initalize PID settings
    PIDSetup.IntializePIDSpark(rightLeader, kVP, kVI, kVD, 1, -1, enc);
    PIDSetup.IntializePIDSpark(leftLeader, kVP, kVI, kVD, 1, -1, enc);
    // get encoders
    rightEnc = rightLeader.getEncoder(EncoderType.kQuadrature, (int) enc.rotationsToPulses(1));
    leftEnc = leftLeader.getEncoder(EncoderType.kQuadrature, (int) enc.rotationsToPulses(1));

    gyro = new AHRS();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double rightOutput, double leftOutput) {
    rightLeader.set(rightOutput);
    leftLeader.set(leftOutput);
  }

  /**
   * @param rightVel - the RPM velocity for the right half of the robot
   * @param leftVel  - the RPM velocity for the left half of the robot
   */
  public void velocityDrive(double rightVel, double leftVel) {
    rightLeader.getPIDController().setReference(rightVel, ControlType.kVelocity);
    leftLeader.getPIDController().setReference(leftVel, ControlType.kVelocity);
  }

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

  public double getHeadingDegrees() {
    return gyro.getAngle();
  }

  public double getRotVelocity() {
    return gyro.getRate();
  }

  public void resetHeading() {
    gyro.reset();
  }

  public void setPID(double kP, double kI, double kD) {
    // right PID configuration
    rightLeader.getPIDController().setP(kP);
    rightLeader.getPIDController().setI(kI);
    rightLeader.getPIDController().setD(kD);
    // left PID configuration
    leftLeader.getPIDController().setP(kP);
    leftLeader.getPIDController().setI(kI);
    leftLeader.getPIDController().setD(kD);
  }

  /**
   * Turns by rotations relative to current location
   */
  public void positionDrive(double rightRotations, double leftRotations) {
    double goalRight = rightRotations + getRightPos();
    double goalLeft = leftRotations + getLeftPos();
    rightLeader.getPIDController().setReference(enc.rotationsToPulses(goalRight), ControlType.kPosition);
    leftLeader.getPIDController().setReference(enc.rotationsToPulses(goalLeft), ControlType.kPosition);

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
