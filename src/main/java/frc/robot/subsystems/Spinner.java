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
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class Spinner extends SubsystemBase {

    // DEFAULT COLOR VARIABLES
    public final String BLUE = "#00FFFF";
    public final String GREEN = "#00FF00";
    public final String RED = "#FF0000";
    public final String YELLOW = "#FFFF00";

    private final CANSparkMax spinner;
    private final CANSparkMax arm;
    private final CANEncoder armAngle;

    private final Encoder enc = Encoder.Grayhill256;

  /*
  BLUE: Cyan 100, Magenta 0, Yellow 0 (00FFFF)
  GREEN: Cyan 100, Magenta 0, Yellow 100 (00FF00)
  RED: Cyan 0, Magenta 100, Yellow 100 (FF0000)
  YELLOW: Cyan 0, Magenta 0, Yellow 100 (FFFF00)

  RED GREEN BLUE YELLOW
  */

  /**
   * Creates a new Spinner subsystem.
   * @param isPressed: User input for whether spinner goes or not
   */
  public Spinner() {
    spinner = new CANSparkMax(Constants.K_SPINNER_MOTOR_ID, MotorType.kBrushless);
    arm = new CANSparkMax(Constants.K_SPINNER_ARM_ID, MotorType.kBrushless);
    armAngle = arm.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
  }

  /**
   * Controls the motor that spins the arm
   */
  public void spin() {
    spinner.set(0.5);
  }

  /**
   * Stops the spinnning
   */
  public void stop() {
    spinner.set(0);
  }

  /**
   * Extends arms???
   */
  public void extend() {
    // TODO arm
    // not sure how to write with motor
    // idea is to rotate 180 or extend
    // pretty sure this is also pneumatic piston
    // not a motor?
  }

  /**
   * Retracts arm
   */
  public void retract() {

  }

  /**
   * Gets the current angle of the arm
   */
  public double getArmAngle() {
    return armAngle.getPosition();
  }

  public void moveArm() {
    // TODO auto arm thingy
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (vision stuff)
  }
}