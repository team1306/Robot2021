<<<<<<< HEAD
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

=======
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
>>>>>>> 099ab437a482949f8815e17406c88f0ff0fe8cc7
import frc.robot.Constants;
import frc.robot.utils.Encoder;
import frc.robot.subsystems.Index;

public class Intake extends SubsystemBase {

<<<<<<< HEAD
    // pretty sure there is only one motor for intake?
    private final CANSparkMax motor;

    // TODO pistons on recad #1 - digital input because they go up and down

    // encoder
    private final Encoder enc = Encoder.Grayhill256;

  /**
   * Creates a new Intake subsystem.
   */
  public Intake() {
      motor = new CANSparkMax(Constants.K_INTAKE_MOTOR_ID, MotorType.kBrushless); 
  }

  /**
   * This method spins the motor so the power cells go in
   */
  public void spin() {
        motor.set(0.5);    
  }

  /**
   * This method spins the motor backwards to un-jam power cells
   */
  public void spit() {
      motor.set(-0.5); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (vision stuff)
  }
=======
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final CANEncoder rightEnc;
    private final CANEncoder leftEnc;

    private final DoubleSolenoid intakeArm;
    private final double armCounterMax = 3;
    private double armCounter = 0;

    private static final Value ExtensionDirection = Value.kReverse;
    private static final Value RetractionDirection = Value.kForward;

    private final Encoder enc = Encoder.Grayhill256;

    public Intake() {
        leftMotor = new CANSparkMax(0, MotorType.kBrushless); // change the can number
        rightMotor = new CANSparkMax(1, MotorType.kBrushless);

        // reset
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        // follow
        leftMotor.follow(rightMotor);

        // idle
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        // encoders
        leftEnc = leftMotor.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));
        rightEnc = rightMotor.getEncoder(EncoderType.kHallSensor, (int) enc.rotationsToPulses(1));

        intakeArm = new DoubleSolenoid(Constants.K_INTAKE_SOLENOID_UP, Constants.K_INTAKE_SOLENOID_DOWN);
        register();
    }

    @Override
    public void periodic() {

    }

    /**
     * Intake
     */
    public void intake(double speed) {
        rightMotor.set(speed);
    }

    /**
     * Extends the intake
     */
    public void extendDown() {
        intakeArm.set(ExtensionDirection);
        armCounter = armCounterMax;
    }

    /**
     * Retracts the intake
     */
    public void retractUp() {
        intakeArm.set(RetractionDirection);
        armCounter = armCounterMax;
    }

>>>>>>> 099ab437a482949f8815e17406c88f0ff0fe8cc7
}