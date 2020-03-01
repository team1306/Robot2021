package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public final VictorSPX indexMotor;
    private final VictorSPX intakeMain;
    private final VictorSPX intakeLeft;

    private final DoubleSolenoid intakeArm;

    private final DigitalInput indexSwitch;

    public Intake() {
        indexMotor = new VictorSPX(Constants.K_INTAKE_INDEXER_ID);
        indexMotor.setInverted(true);
        intakeMain = new VictorSPX(Constants.K_INTAKE_AXEL_RIGHT_ID);
        intakeLeft = new VictorSPX(Constants.K_INTAKE_AXEL_LEFT_ID);
        intakeLeft.setInverted(true);
        indexSwitch = new DigitalInput(Constants.K_INTAKE_INDEX_SWITCH);

        intakeArm = new DoubleSolenoid(Constants.K_INTAKE_SOLENOID_UP,Constants.K_INTAKE_SOLENOID_DOWN);
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
     * Returns whether the indexing sensor detects a ball
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