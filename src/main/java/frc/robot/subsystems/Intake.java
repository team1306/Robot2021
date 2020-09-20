/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class Intake extends SubsystemBase {

  // Two motors for the recad robot intake
  private final CANSparkMax motorRight;
  private final CANSparkMax motorLeft;

  // TODO pistons on recad #1 - digital input because they go up and down

  // encoder
  private final Encoder enc = Encoder.Grayhill256;

  /**
   * Creates a new Intake subsystem.
   */
  public Intake() {
    motorRight = new CANSparkMax(Constants.K_INTAKE_MOTOR_ID, MotorType.kBrushless);
    motorLeft = new CANSparkMax(Constants.K_INTAKE_MOTOR_ID, MotorType.kBrushless);

    // reset
    motorRight.restoreFactoryDefaults();
    motorLeft.restoreFactoryDefaults();

    // follow
    motorRight.follow(motorLeft);

    // idle
    motorRight.setIdleMode(IdleMode.kBrake);
    motorLeft.setIdleMode(IdleMode.kBrake);
  }

  /**
   * This method spins the motor so the power cells go in
   */
  public void spin() {
    motorRight.set(0.5);
  }

  /**
   * This method spins the motor backwards to un-jam power cells
   */
  public void spit() {
    motorRight.set(-0.5);
  }

  public void spin(double speed) {
    motorRight.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (vision stuff)
  }
}