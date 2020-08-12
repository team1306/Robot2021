/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class DriveTrain extends SubsystemBase {

  // TODO Delete this??
  // declares instance variables (motors for each wheel and two encoders)
  private final CANSparkMax rightFront;
  private final CANSparkMax rightMid;
  private final CANSparkMax rightBack;
  private final CANSparkMax leftFront;
  private final CANSparkMax leftMid;
  private final CANSparkMax leftBack;

  private final CANEncoder rightEnc;
  private final CANEncoder leftEnc;

  private final Encoder enc = Encoder.Grayhill256;

  /**
   * Creates a new DriveTrain subsystem.
   */
  public DriveTrain() {

    // initialize motors
    rightFront = new CANSparkMax(Constants.K_DRIVE_RIGHT_FRONT_ID, MotorType.kBrushless);
    rightMid = new CANSparkMax(Constants.K_DRIVE_RIGHT_MIDDLE_ID, MotorType.kBrushless);
    rightBack = new CANSparkMax(Constants.K_DRIVE_RIGHT_BACK_ID, MotorType.kBrushless);

    leftFront = new CANSparkMax(Constants.K_DRIVE_LEFT_FRONT_ID, MotorType.kBrushless);
    leftMid = new CANSparkMax(Constants.K_DRIVE_LEFT_MIDDLE_ID, MotorType.kBrushless);
    leftBack = new CANSparkMax(Constants.K_DRIVE_LEFT_BACK_ID, MotorType.kBrushless);

    // reset
    rightFront.restoreFactoryDefaults();
    rightMid.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();

    leftFront.restoreFactoryDefaults();
    leftMid.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();

    // follow
    rightMid.follow(rightFront);
    rightBack.follow(rightFront);

    leftMid.follow(leftFront);
    leftMid.follow(leftFront);

    // idle
    rightFront.setIdleMode(IdleMode.kBrake);
    rightMid.setIdleMode(IdleMode.kBrake);
    rightBack.setIdleMode(IdleMode.kBrake);

    leftFront.setIdleMode(IdleMode.kBrake);
    leftMid.setIdleMode(IdleMode.kBrake);
    leftBack.setIdleMode(IdleMode.kBrake);

    // encoders
    rightEnc = rightFront.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
    leftEnc = leftFront.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));

  }

  /**
   * Sets the front of each side's wheels to values from driver 
   * One stick controls one side of the robot's wheels
   */
  public void tankDrive(double rightOutput, double leftOutput) {
    rightFront.set(rightOutput);
    leftFront.set(-leftOutput);
  }

  /**
   * Sets the front of each side's wheels to values from the driver
   * Speed and rotation are assigned to separate joysticks
   * @param velocity input of robot speed
   * @param rotation input of robot rotation
   */
  public void arcadeDrive(double velocity, double rotation) {
      rightFront.set(velocity + rotation);
      leftFront.set(-velocity + rotation);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run (vision stuff)
  }

  public double getRightRPM() {
    return rightEnc.getVelocity();
  }

  public double getLeftRPM() {
    return leftEnc.getVelocity();
  }

  public double getRightPos() {
    return rightEnc.getPosition();
  }

  public double getLeftPos() {
    return leftEnc.getPosition();
  }
}