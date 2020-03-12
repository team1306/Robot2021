package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.DriveTrain;

public class PositionVisionPIDController extends PIDController {

    private final double maxVisionTurn = 1.0;

    private DriveTrain driveTrain;

    private Notifier looper;
    private LinearFilter filter;

    private double goalHeading = 0;
    private double period;
    private boolean isRunning = false;

    /**
     * Constructs the PID Controller with the given constants and with a reference
     * to the drivetain object.
     * 
     * @param Kp         - p gain
     * @param Ki         - i gain
     * @param Kd         - d gain
     * @param period     - the refresh period to loop at, in milleseconds
     * @param driveTrain - drivetrain instance
     */
    public PositionVisionPIDController(double Kp, double Ki, double Kd, double period, DriveTrain driveTrain) {
        super(Kp, Ki, Kd, period);

        this.period = period;
        this.driveTrain = driveTrain;

        looper = new Notifier(this::runLoop);
        filter = LinearFilter.singlePoleIIR(7 * period, period);
    }

    /**
     * Execute the calculations for the PID loop and put the output to the
     * drivetrain
     */
    private void runLoop() {
        this.setSetpoint(filter.calculate(goalHeading));
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
        looper.startPeriodic(period / 1000);
        isRunning = true;
    }

    /**
     * Starts the loop notifier running with a target goal heading.
     * 
     * @param goalHeading
     */
    public void start(double goalHeading) {
        resetGoalHeading(goalHeading);
        looper.startPeriodic(period / 1000);
        isRunning = true;
    }

    /**
     * Stops the loop from calculating and writing to DriveTrain
     */
    public void stop() {
        looper.stop();
        isRunning = false;
        driveTrain.tankDrive(0, 0);
        this.reset();
        filter.reset();
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
        goalHeading = nearestHeadingEquivalent(heading);
    }

    /**
     * Set the heading and skip the averaging- ignore past results. DOES still
     * translate heading to nearest equivalent.
     */
    private void resetGoalHeading(double heading) {
        goalHeading = nearestHeadingEquivalent(heading);
        this.setSetpoint(goalHeading);
        filter.reset();
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
        double relative = heading - curr;
        relative = floormod(relative, 360);
        if (relative > 180) {
            relative = relative - 360;
        }
        double nearest = relative + curr;
        return nearest;
    }

    private double floormod(double num, double denom) {
        return ((num % denom) + denom) % denom;
    }
}