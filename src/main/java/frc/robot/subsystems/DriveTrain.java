/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.RobotMap;

/**
 * Drivetrain subsystem to drive the robot. Uses DifferentialDrive
 */

public class DriveTrain extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonFX leftLeader;
  private TalonFX rightLeader;

  private TalonFX leftFollower;
  private TalonFX rightFollower;

  //public SensorCollection leftSensors;
  //public SensorCollection rightSensors;

  private DifferentialDrive tankDrive;

  public static boolean reverse = false;

  public DriveTrain() {
    // Initialize motors
    leftLeader = new TalonFX(Constants.K_DRIVE_FRONT_LEFT_ID);
    rightLeader = new TalonFX(Constants.K_DRIVE_FRONT_RIGHT_ID);

    leftFollower = new TalonFX(Constants.K_DRIVE_BACK_LEFT_ID);
    rightFollower = new TalonFX(Constants.K_DRIVE_BACK_RIGHT_ID);

    leftLeader.configFactoryDefault();
    rightLeader.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightFollower.configFactoryDefault();

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // Initialize sensor values
    //leftSensors = leftLeader.getSensorCollection();
    //rightSensors = rightLeader.getSensorCollection();

    // Initialize DifferentialDrive object for use later
    //tankDrive = new DifferentialDrive(leftLeader, rightLeader);
    //tankDrive.setDeadband(0.05);
  }

  /**
   * Pushes up {@link DifferentialDrive}.tankDrive
   * 
   * DifferentialDrive documentation: Tank drive method for differential drive
   * platform. The calculated values will be squared to decrease sensitivity at
   * low speeds.
   *
   * @param left  The robot's left side speed along the X axis [-1.0..1.0].
   *              Forward is positive.
   * @param right The robot's right side speed along the X axis [-1.0..1.0].
   *              Forward is positive.
   */
  public void drive(double left, double right) {
    if (reverse) {
      left = -left;
      right = -right;
    }
    System.out.println("Drive Train drive is running.");

    leftLeader.set(ControlMode.PercentOutput, left);
    rightLeader.set(ControlMode.PercentOutput, right);
  }

  /**
   * Pushes up edu.wpi.first.wpilibj.drive.DifferentialDrive.arcadeDrive
   * 
   * DifferentailDrive documentation: Arcade drive method for differential drive
   * platform. The calculated values will be squared to decrease sensitivity at
   * low speeds.
   *
   * @param speed    The robot's speed along the X axis [-1.0..1.0]. Forward is
   *                 positive.
   * @param rotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                 Clockwise is positive.
   */
  public void arcadeDrive(double speed, double rotation) {
    if (reverse) {
      speed = -speed;
    }
    double speedMultiplyer=(SmartDashboard.getNumber("DB/Slider 0", 5)/5)*0.4+0.6;
    double turnMultiplyer=(SmartDashboard.getNumber("DB/Slider 0", 5)/5)*0.2+0.8;
    speed=speed*speedMultiplyer;
    rotation=rotation*turnMultiplyer;
    tankDrive.arcadeDrive(speed, rotation);
  }
}