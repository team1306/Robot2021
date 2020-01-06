/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private final TalonSRX rightLeader;
  private final TalonSRX leftLeader;

  // PID Tuning constants
  private static final int kLoopType = 0;// 0 = closed loop, 1 = auxilary closed loop
  private static final FeedbackDevice kSensorType = FeedbackDevice.QuadEncoder;
  private static final int kTimeout = 0; // Set to any positive non-0 number to check success
  private static final double kP = 0.3;
  private static final double kI = 0.01;
  private static final double kD = 0;

  public DriveTrain() {
    // initialize motor controllers
    // right
    rightLeader = new TalonSRX(Constants.K_DRIVE_RIGHT_FRONT_ID);
    TalonSRX rightFollower = new TalonSRX(Constants.K_DRIVE_RIGHT_BACK_ID);
    // left
    leftLeader = new TalonSRX(Constants.K_DRIVE_LEFT_FRONT_ID);
    TalonSRX leftFollower = new TalonSRX(Constants.K_DRIVE_LEFT_BACK_ID);
    // set follow
    rightFollower.follow(rightLeader);
    leftFollower.follow(leftLeader);
    // initalize PID settings
    // right sensor configuration
    rightLeader.configFactoryDefault();
    rightLeader.configSelectedFeedbackSensor(kSensorType, kLoopType, kTimeout);
    rightLeader.configNominalOutputForward(0, kTimeout);// nominal = minimum output
    rightLeader.configNominalOutputReverse(0, kTimeout);
    rightLeader.configPeakOutputForward(1, kTimeout);// peak = maximum output
    rightLeader.configPeakOutputReverse(1, kTimeout);
    rightLeader.setSensorPhase(false);
    // left sensor configuration
    leftLeader.configFactoryDefault();
    leftLeader.configSelectedFeedbackSensor(kSensorType, kLoopType, kTimeout);
    leftLeader.configNominalOutputForward(0, kTimeout);// nominal = minimum output
    leftLeader.configNominalOutputReverse(0, kTimeout);
    leftLeader.configPeakOutputForward(1, kTimeout);// peak = maximum output
    leftLeader.configPeakOutputReverse(1, kTimeout);
    leftLeader.setSensorPhase(false);
    leftFollower.setInverted(true);
    leftLeader.setInverted(true);
    // right PID configuration
    rightLeader.config_kP(kLoopType, kP, kTimeout);
    rightLeader.config_kI(kLoopType, kI, kTimeout);
    rightLeader.config_kD(kLoopType, kD, kTimeout);
    // left PID configuration
    leftLeader.config_kP(kLoopType, kP, kTimeout);
    leftLeader.config_kI(kLoopType, kI, kTimeout);
    leftLeader.config_kD(kLoopType, kD, kTimeout);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double rightOutput, double leftOutput) {
    rightLeader.set(ControlMode.PercentOutput, rightOutput);
    leftLeader.set(ControlMode.PercentOutput, leftOutput);
    System.out.println("Commanded output: " + leftOutput);
    System.out.println("Actual Output:" + leftLeader.getMotorOutputPercent() + ", Right Percent: "
        + rightLeader.getMotorOutputPercent());

  }

  /**
   * @param rightVel - the RPM velocity for the right half of the robot
   * @param leftVel  - the RPM velocity for the left half of the robot
   */
  public void velocityDrive(double rightVel, double leftVel) {
    rightLeader.set(ControlMode.Velocity, rightVel);
    leftLeader.set(ControlMode.Velocity, leftVel);
  }

  public double getRightRPM() {
    return rightLeader.getSelectedSensorVelocity(kLoopType);
  }

  public double getLeftRPM() {
    return leftLeader.getSelectedSensorVelocity(kLoopType);
  }

  public void setPID(double kP, double kI, double kD) {
    // right PID configuration
    rightLeader.config_kP(kLoopType, kP, kTimeout);
    rightLeader.config_kI(kLoopType, kI, kTimeout);
    rightLeader.config_kD(kLoopType, kD, kTimeout);
    // left PID configuration
    leftLeader.config_kP(kLoopType, kP, kTimeout);
    leftLeader.config_kI(kLoopType, kI, kTimeout);
    leftLeader.config_kD(kLoopType, kD, kTimeout);
  }
}
