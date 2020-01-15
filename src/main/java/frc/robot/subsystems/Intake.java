package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final Spark indexMotor;
    private final Spark intakeMotor;

    private final DigitalInput indexSwitch;

    public Intake() {
        indexMotor = new Spark(Constants.K_INTAKE_INDEXER_SPARK);
        intakeMotor = new Spark(Constants.K_INTAKE_GRABBER_SPARK);

        indexSwitch = new DigitalInput(Constants.K_INTAKE_INDEX_SWITCH);
    }

    /**
     * Indexes a ball once intaked.
     * 
     * @param speed - double from 1 (in) to -1 (out)
     */
    public void index(double speed) {
        indexMotor.set(speed);
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
     * @param double speed -
     */
    public void intake(double speed) {
        intakeMotor.set(speed);
    }

}