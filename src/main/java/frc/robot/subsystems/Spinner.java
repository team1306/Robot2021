package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spinner extends SubsystemBase {
    
    private final TalonSRX spinnerMotor;

    public Spinner(){
        //make the spinner
        spinnerMotor = new TalonSRX(Constants.K_SPINNER_MOTOR_ID);
    }

    public void spin(double output){
        spinnerMotor.set(ControlMode.PercentOutput, output);
    }

}