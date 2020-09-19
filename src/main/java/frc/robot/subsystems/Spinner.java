/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class Spinner extends SubsystemBase {

    // DEFAULT COLOR VARIABLES
    public final String BLUE = "#00FFFF";
    public final String GREEN = "#00FF00";
    public final String RED = "#FF0000";
    public final String YELLOW = "#FFFF00";

    private final CANSparkMax spinner;

    private final DigitalInput isPressed;

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
  public Spinner(DigitalInput isPressed) {
    this.isPressed = isPressed;
    spinner = new CANSparkMax(Constants.K_SPINNER_MOTOR_ID, MotorType.kBrushless);
  }

  /**
   * Spins the motor until the driver releases the button
   */
  public void spin() {
    while(this.isPressed.get()) {
      spinner.set(0.5);
    }
  }

  /**
   * Meant for if the camera is the one detecting the color
   * TODO auto for spinner
   */
  public void autoCamera() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (vision stuff)
  }

}