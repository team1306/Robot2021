package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    
    TalonSRX motor;

    public Climber(){
        motor = new TalonSRX(Constants.K_CLIMBER_MOTOR_ID);
    }

    public void climb(double speed){
        motor.set(ControlMode.PercentOutput, speed);
    }

}