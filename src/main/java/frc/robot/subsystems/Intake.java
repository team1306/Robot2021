package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final TalonSRX indexMotor;
    private final VictorSPX intakeMain;
    private final VictorSPX intakeLeft;

    private final DigitalInput indexSwitch;

    public Intake() {
        indexMotor = new TalonSRX(Constants.K_INTAKE_INDEXER_ID);
        intakeMain = new VictorSPX(Constants.K_INTAKE_AXEL_RIGHT_ID);
        intakeLeft = new VictorSPX(Constants.K_INTAKE_AXEL_LEFT_ID);
        indexSwitch = new DigitalInput(Constants.K_INTAKE_INDEX_SWITCH);
    }

    /**
     * Indexes a ball once intaked.
     * 
     * @param speed - double from 1 (in) to -1 (out)
     */
    public void index(double speed) {
        indexMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Returns whether the indexing switch is pressed
     */
    public boolean getSwitch() {
        return indexSwitch.get();
    }

    /**
     * runs the intake bar
     * 
     * @param speedRight -The speed of the right half-axle and the roller bar
     * @param speedLeft -The speed of the left half-axle 
     */
    public void intake(double speedRight, double speedLeft) {
        intakeMain.set(ControlMode.PercentOutput, speedRight);
        intakeLeft.set(ControlMode.PercentOutput, speedLeft);
    }

}