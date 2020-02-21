package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.DriveTrain;

public class PositionVisionPIDController extends PIDController {

    private final double maxVisionTurn = 0.5;

    private DriveTrain driveTrain;

    private Notifier looper;

    private double goalHeading = 0;
    private double period;
    private boolean isRunning = false;

    public PositionVisionPIDController(double Kp, double Ki, double Kd, double period, DriveTrain driveTrain) {
        super(Kp, Ki, Kd, period);

        this.period = period;
        this.driveTrain = driveTrain;

        this.enableContinuousInput(0, 360);

        looper = new Notifier(this::runLoop);
    }

    /**
     * Execute the calculations for the PID loop and put the output to the
     * drivetrain
     */
    private void runLoop() {
        double out = this.calculate(driveTrain.getHeadingDegrees());
        out = Math.min(Math.max(-maxVisionTurn, out), maxVisionTurn);// clamp to range
        driveTrain.tankDrive(-out, out);
    }

    /**
     * Starts the loop notifier running.
     * 
     * This call will also change the goal heading to the closest equivalent
     * heading, i.e. a goal heading of 360 degrees with a current heading of 10
     * degrees will be changed to a goal heading of 0 degrees.
     */
    public void start() {
        resetGoalHeading(this.goalHeading);// reset closest path
        looper.startPeriodic(period);
        isRunning = true;
    }

    /**
     * Starts the loop notifier running with a target goal heading.
     * 
     * @param goalHeading
     */
    public void start(double goalHeading) {
        resetGoalHeading(goalHeading);
        looper.startPeriodic(period);
        isRunning = true;
    }

    /**
     * Stops the loop from calculating and writing to DriveTrain
     */
    public void stop() {
        looper.stop();
        isRunning = false;
    }

    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Updates the loop goal point.
     * 
     * Does NOT start the loop
     * 
     * @param heading- goal gyro value, not relative to current robot heading
     */
    public void setGoalHeading(double heading) {
        goalHeading = 0.5 * (heading + nearestHeadingEquivalent(heading));
        /*
         * Breakdown of above function: To smooth results, we average it with previous
         * results. However, if the previous result went through this process, we then
         * also have the average of all previous inputs a different weights. For term n,
         * where n is how many terms away from the current input, the weight is equal to
         * 0.5/(2^n); this has a sum of one, because of the sum of infinite geometric
         * series (the lost terms are made up by the reset).
         * 
         */
        this.setSetpoint(goalHeading);
    }

    /**
     * Set the heading and skip the averaging- ignore past results. DOES still
     * translate heading to nearest equivalent.
     */
    private void resetGoalHeading(double heading) {
        goalHeading = nearestHeadingEquivalent(heading);
        this.setSetpoint(goalHeading);
    }

    /**
     * Converts a heading into the nearest equivalent heading to the robots current
     * heading.
     * 
     * For instance, if the robot is at a heading of 270 degrees, with an input of
     * zero, it would return 360 because a heading of 360 is the same as a heading
     * of 0 but has less degrees between the current heading, meaning the shortest
     * turn.
     */
    private double nearestHeadingEquivalent(double heading) {
        double curr = driveTrain.getHeadingDegrees();
        double r = 360 * (int) (curr / 360);
        double fcurr = curr - r;// remove the number of whole rotation is the same as mod
        double fheading = floormod(heading, 360);
        if (floormod(fheading - fcurr, 360) > 180) {
            r -= 360;
        }
        return r + fheading;
    }

    private double floormod(double num, double denom) {
        return ((num % denom) + denom) % denom;
    }
}