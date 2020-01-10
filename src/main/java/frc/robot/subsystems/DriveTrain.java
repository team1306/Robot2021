/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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

  // PID Tuning constants
  private static final double kP = 0.3;
  private static final double kI = 0.01;
  private static final double kD = 0;

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
    PIDSetup.IntializePIDSpark(rightLeader, kP, kI, kD, 1, enc);
    PIDSetup.IntializePIDSpark(leftLeader, kP, kI, kD, 1, enc);
    // get encoders
    rightEnc = rightLeader.getEncoder(EncoderType.kQuadrature, (int) enc.rotationsToPulses(1));
    leftEnc = leftLeader.getEncoder(EncoderType.kQuadrature, (int) enc.rotationsToPulses(1));
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
}
