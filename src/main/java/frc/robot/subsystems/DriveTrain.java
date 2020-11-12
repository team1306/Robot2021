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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class DriveTrain extends SubsystemBase {

  // TODO Delete this??
  // declares instance variables (motors for each wheel and two encoders)
  private final WPI_TalonSRX rightFront;
  private final WPI_TalonSRX rightMid;
  private final WPI_TalonSRX rightBack;
  private final WPI_TalonSRX leftFront;
  private final WPI_TalonSRX leftMid;
  private final WPI_TalonSRX leftBack;

  /**
   * Creates a new DriveTrain subsystem.
   */
  public DriveTrain() {

    // initialize motors
    rightFront = new WPI_TalonSRX(Constants.K_DRIVE_RIGHT_FRONT_ID);
    rightMid = new WPI_TalonSRX(Constants.K_DRIVE_RIGHT_MIDDLE_ID);
    rightBack = new WPI_TalonSRX(Constants.K_DRIVE_RIGHT_BACK_ID);

    leftFront = new WPI_TalonSRX(Constants.K_DRIVE_LEFT_FRONT_ID);
    leftMid = new WPI_TalonSRX(Constants.K_DRIVE_LEFT_MIDDLE_ID);
    leftBack = new WPI_TalonSRX(Constants.K_DRIVE_LEFT_BACK_ID);

    rightMid.follow(rightFront);
    rightBack.follow(rightFront);

    leftMid.follow(leftFront);
    leftBack.follow(leftBack);

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
}