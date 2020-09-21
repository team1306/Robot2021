/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private final CANSparkMax climber;

  /**
   * Creates a new Climber subsystem.
   */
  public Climber(DigitalInput isClimbing, DigitalInput isPulling) {
    climber = new CANSparkMax(Constants.K_CLIMBER_MOTOR_ID, MotorType.kBrushless);
  }

  /**
   * Spins the motor until the driver releases the button
   */

  public void unfold() {
    // pnematic piston work?
  }

  public void extend() {
    climber.set(0.5);
  }

  /**
   * 
   */
  public void retract() {
    climber.set(-0.5);
  }

  /**
   * Stops the climber motor from moving
   */
  public void stop() {
    climber.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run (vision stuff)
  }

}