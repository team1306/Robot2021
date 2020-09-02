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
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.utils.Encoder;

public class Index extends SubsystemBase {

    //contains true if the position has a ball
    private boolean[] ballPositions;
    
    //move to constants?
    private int positionsInIndex = 5;
    private int entrancePosition = 0;
    private int exitPosition = 2;

    private final CANSparkMax spinner;
    private final CANEncoder spinnerEnc;
    private final com.revrobotics.CANPIDController pidController;

    private final Encoder enc = Encoder.Grayhill256;

    /**
     * creates a new index object
     */
  public Index() {
      ballPositions = new boolean[positionsInIndex];
      spinner = new CANSparkMax(Constants.K_INDEX_MOTOR_ID,MotorType.kBrushless);
      spinnerEnc = spinner.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
      spinnerEnc.setPosition(0.0);

      //TODO constants need to be tuned
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
    boolean storage = ballPositions[0];

    for(int i = positionsInIndex - 1; i > 0; i--) {
        ballPositions[i] = ballPositions[i - 1];
    }

    ballPositions[positionsInIndex - 1] = storage;

    double currentPosition = spinnerEnc.getPosition();

    pidController.setReference((.2 + spinnerEnc.getPosition()) % 1, ControlType.kPosition);
  }


  /**
   * @return true if extrance index is open
   */
  public boolean canIntake() {
    return !ballPositions[entrancePosition];
 }

  /**
   * TODO english
   * adds a intaked ball into the index
   */
  public void intake() {
      if(canIntake()) {
          ballPositions[entrancePosition] = true;
      }
  }


  /**
   * @return true if ball in exit index
   */
  public boolean canShoot() {
    return ballPositions[exitPosition];
  }

  /**
   * removes a shot ball from the index
   */
  public void shoot() {
      if(canShoot()) {
          ballPositions[exitPosition] = false;
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (vision stuff)
  }
}