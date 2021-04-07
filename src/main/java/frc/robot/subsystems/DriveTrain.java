/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

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

  private TalonFX angleMotorFrontLeft;
  private TalonFX angleMotorFrontRight;
  private TalonFX angleMotorBackLeft;
  private TalonFX angleMotorBackRight;

  private AHRS navx;

  private final DifferentialDriveOdometry m_odometry;

  //public SensorCollection leftSensors;
  //public SensorCollection rightSensors;

  private DifferentialDrive tankDrive;

  public static boolean reverse = true;
  private double oldTicksLeft;
  private double oldTicksRight;

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

    angleMotorFrontLeft = new TalonFX(Constants.K_TURN_FRONT_LEFT_ID);
    angleMotorFrontRight = new TalonFX(Constants.K_TURN_FRONT_RIGHT_ID);
    angleMotorBackLeft = new TalonFX(Constants.K_TURN_BACK_LEFT_ID);
    angleMotorBackRight = new TalonFX(Constants.K_TURN_BACK_RIGHT_ID);

    angleMotorFrontLeft.configFactoryDefault();
    angleMotorFrontRight.configFactoryDefault();
    angleMotorBackLeft.configFactoryDefault();
    angleMotorBackRight.configFactoryDefault();

    angleMotorFrontLeft.setNeutralMode(NeutralMode.Brake);
    angleMotorFrontRight.setNeutralMode(NeutralMode.Brake);
    angleMotorBackLeft.setNeutralMode(NeutralMode.Brake);
    angleMotorBackRight.setNeutralMode(NeutralMode.Brake);
    // Initialize sensor values
    //leftSensors = leftLeader.getSensorCollection();
    //rightSensors = rightLeader.getSensorCollection();

    // Initialize DifferentialDrive object for use later
    //tankDrive = new DifferentialDrive(leftLeader, rightLeader);
    //tankDrive.setDeadband(0.05);\
    
    // m_leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);

    // resetEncoders();
    m_odometry = new DifferentialDriveOdometry(navx.getRotation2d());
    navx = new AHRS(Port.kMXP);
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

    leftLeader.set(ControlMode.PercentOutput, -left);
    rightLeader.set(ControlMode.PercentOutput, right);
  }

  public void driveArcade(double rotation, double forward, double backward) {
    double turn = forward - backward;
  }

  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

    // Square the inputs (while preserving the sign) to increase fine control
    // while permitting full power.
    if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
    }

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

    leftLeader.set(ControlMode.PercentOutput, MathUtil.clamp(leftMotorOutput, -1.0, 1.0));
    double maxOutput = 1;
    rightLeader.set(ControlMode.PercentOutput, MathUtil.clamp(rightMotorOutput, -1.0, 1.0));

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

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    double leftOutput = leftVolts / Constants.MAX_VOLTS;
    double rightOutput = -rightVolts / Constants.MAX_VOLTS;

    leftLeader.set(ControlMode.PercentOutput, leftOutput);
    rightLeader.set(ControlMode.PercentOutput, rightOutput);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftLeader.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse * 10, 
                                            rightLeader.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse * 10);
  } // the 10 is to compensate for the 100ms from getSelectedSensorVelocity() 

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    double currentTicksLeft = leftLeader.getSelectedSensorPosition();
    double deltaDistanceTicksLeft = currentTicksLeft - oldTicksLeft;
    oldTicksLeft = currentTicksLeft;

    double currentTicksRight = rightLeader.getSelectedSensorPosition();
    double deltaDistanceTicksRight = currentTicksRight - oldTicksRight;
    oldTicksRight = currentTicksRight;

    m_odometry.update(
        navx.getRotation2d(), 
        deltaDistanceTicksLeft * Constants.kEncoderDistancePerPulse, 
        deltaDistanceTicksRight * Constants.kEncoderDistancePerPulse);
  }
}