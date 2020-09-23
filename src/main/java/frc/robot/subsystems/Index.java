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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import frc.robot.Constants;
import frc.robot.utils.Encoder;

//TODO add jiggling 

public class Index extends SubsystemBase {


    private final CANSparkMax spinner;
    private final CANEncoder spinnerEnc;
    private final com.revrobotics.CANPIDController pidController;

    private final Encoder enc = Encoder.Grayhill256;

    /**
     * creates a new index object
     */
  public Index() {
      spinner = new CANSparkMax(Constants.K_INDEX_MOTOR_ID,MotorType.kBrushless);
      spinnerEnc = spinner.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
      spinnerEnc.setPosition(0.0);

      //TODO constants need to be tuned
      //file specific
      pidController = spinner.getPIDController();
      pidController.setP(Constants.KP);
      pidController.setI(Constants.KI);
      pidController.setD(Constants.KD);
  }

  /**
   * TODO 
   * Moves forward one index position
   */
  public void rotate() {

    pidController.setReference((.2 + spinnerEnc.getPosition()) % 1, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (vision stuff)
  }
}