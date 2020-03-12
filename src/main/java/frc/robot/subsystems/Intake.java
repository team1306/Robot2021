package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private static final Value ExtensionDirection = Value.kReverse;
    private static final Value RetractionDirection = Value.kForward;

    public final VictorSPX indexMotor;
    private final VictorSPX intakeMain;
    private final VictorSPX intakeLeft;

    private final DoubleSolenoid intakeArm;
    private final double armCounterMax = 3;
    private double armCounter=0;

    private final DigitalInput indexBottom;
    private final DigitalInput indexTop;


    public Intake() {
        indexMotor = new VictorSPX(Constants.K_INTAKE_INDEXER_ID);
        indexMotor.setInverted(true);
        intakeMain = new VictorSPX(Constants.K_INTAKE_AXEL_RIGHT_ID);
        intakeLeft = new VictorSPX(Constants.K_INTAKE_AXEL_LEFT_ID);
        intakeLeft.setInverted(true);
        indexTop = new DigitalInput(Constants.K_INTAKE_INDEX_SWITCH_TOP);
        indexBottom = new DigitalInput(Constants.K_INTAKE_INDEX_SWITCH_BOTTOM);
        intakeArm = new DoubleSolenoid(Constants.K_INTAKE_SOLENOID_UP, Constants.K_INTAKE_SOLENOID_DOWN);
        register();
    }

    @Override
    public void periodic() {
        if(armCounter == 0){
            intakeArm.set(Value.kOff);
        }else{
            armCounter-=1;
        }
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
     * Returns whether the indexing sensor detects a ball. If intake is up, the
     * sensor readings are looking at the robot, so will only return false.
     */
    public boolean getSwitchBottom() {
        return (!indexBottom.get()) && isIntakeDown();
    }

    public boolean getSwitchTop(){
        return !indexTop.get();
    }

    /**
     * Retracts the intake
     */
    public void retract() {
        intakeArm.set(RetractionDirection);
        armCounter = armCounterMax;
    }

    /**
     * Extends the intake
     */
    public void extend() {
        intakeArm.set(ExtensionDirection);
        armCounter = armCounterMax;
    }

    public boolean isIntakeDown() {
        return intakeArm.get().equals(ExtensionDirection);
    }

    /**
     * runs the intake bar
     * 
     * @param speedRight -The speed of the right half-axle and the roller bar
     * @param speedLeft  -The speed of the left half-axle
     */
    public void intake(double speedRight, double speedLeft) {
        intakeMain.set(ControlMode.PercentOutput, speedRight);
        intakeLeft.set(ControlMode.PercentOutput, speedLeft);
    }

}