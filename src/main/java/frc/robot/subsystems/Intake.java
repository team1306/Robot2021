package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    private final TalonSRX intakeMotor;

    public Intake(){
        //make the spinner
        intakeMotor = new TalonSRX(Constants.K_INTAKE);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void spin(boolean output, boolean input){
        if(output) {
            intakeMotor.set(ControlMode.PercentOutput, -.25);
        } else if(input) {
            intakeMotor.set(ControlMode.PercentOutput, .25);
        }else {
            intakeMotor.set(ControlMode.PercentOutput, 0);
        }
            
    }

    public double getEncoderPosition(){
        //return intakeMOtor.getSelectedSensorPosition();
        return 0;
    }

}