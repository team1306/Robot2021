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

public class Intake extends SubsystemBase {

    // pretty sure there is only one motor for intake?
    private final CANSparkMax motor;

    // TODO pistons on recad #1 - digital input because they go up and down

    // encoder
    private final Encoder enc = Encoder.Grayhill256;

  /**
   * Creates a new Intake subsystem.
   */
  public Intake() {
      motor = new CANSparkMax(DEVICE_ID, TYPE) //TODO "make" deviceID and type
  }

  /**
   * This method spins the motor so the power cells go in
   */
  public void spin(boolean isIntakeOn) {
      while(isIntakeOn) {
        motor.set(0.5);
      }
  }

  /**
   * This method spins the motor backwards to un-jam power cells
   */
  public void spit(boolean isIntakeJammed) {
      while(isIntakeJammed) {
          motor.set(-0.5);
      }
  }

  /**
   * This method lifts the intake up
   * TODO define max height of lift in rotations?
   */
  public void lift() {
    while()
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (vision stuff)
  }
}