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
import frc.robot.utils.Encoder;
import frc.robot.utils.PIDSetup;

public class DriveTrain extends SubsystemBase {

  private final TalonSRX rightLeader;
  private final TalonSRX leftLeader;
  private final Encoder enc = Encoder.Grayhill256;

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
    PIDSetup.IntializePID(rightLeader, kP, kI, kD, 0, 1, kSensorType, kLoopType, kTimeout);
    PIDSetup.IntializePID(leftLeader , kP, kI, kD, 0, 1, kSensorType, kLoopType, kTimeout);
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
    rightLeader.set(ControlMode.Velocity, enc.RPMtoPIDVelocity(rightVel));
    leftLeader.set(ControlMode.Velocity, enc.RPMtoPIDVelocity(leftVel));
  }

  public double getRightRPM() {
    return enc.PIDVelocityToRPM(rightLeader.getSelectedSensorVelocity(kLoopType));
  }

  public double getLeftRPM() {
    return enc.PIDVelocityToRPM(leftLeader.getSelectedSensorVelocity(kLoopType));
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
