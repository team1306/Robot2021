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

        looper = new Notifier(this::runLoop);
    }

    /**
     * Execute the calculations for the PID loop and put the output to the
     * drivetrain
     */
    private void runLoop() {
        double out = this.calculate(driveTrain.getHeadingDegrees());
        out = Math.min(Math.max(-maxVisionTurn, out), maxVisionTurn);//clamp to range
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
        setGoalHeading(this.goalHeading);// reset closest path
        looper.startPeriodic(period);
        isRunning = true;
    }

    /**
     * Starts the loop notifier running with a target goal heading.
     * 
     * @param goalHeading
     */
    public void start(double goalHeading) {
        setGoalHeading(goalHeading);
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
     * @param heading
     */
    public void setGoalHeading(double heading) {
        goalHeading = heading % 360 + 360 * (int) (driveTrain.getHeadingDegrees() / 360);
        this.setSetpoint(heading);
    }

}