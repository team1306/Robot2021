package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class PositionVisionPIDController extends PIDController {

    private final double maxVisionTurn = 1.0;

    private DriveTrain driveTrain;

    private Notifier looper;

    private double goalHeading = 0;
    private double period;
    private boolean isRunning = false;

    private final TrajectoryConfig trajectoryConfig;
    private Trajectory trajectory;
    private Pose2d startPose;
    private Pose2d endPose;

    /**
     * Constructs the PID Controller with the given constants and with a reference to the drivetain object.
     * @param Kp - p gain
     * @param Ki - i gain
     * @param Kd - d gain
     * @param period - the refresh period to loop at, in milleseconds
     * @param driveTrain - drivetrain instance
     */
    public PositionVisionPIDController(double Kp, double Ki, double Kd, double period, DriveTrain driveTrain) {
        super(Kp, Ki, Kd, period);

        this.period = period;
        this.driveTrain = driveTrain;

        looper = new Notifier(this::runLoop);

        trajectoryConfig = new TrajectoryConfig(1, 2);
        trajectoryConfig.setEndVelocity(0);
        trajectoryConfig.setStartVelocity(0);
        trajectoryConfig.setKinematics(new DifferentialDriveKinematics(Constants.K_TRACK_WIDTH_METERS));

        startPose = new Pose2d(0, 0, new Rotation2d(driveTrain.getHeadingDegrees()));
        endPose = new Pose2d(0, 0, new Rotation2d(driveTrain.getHeadingDegrees()));
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
        looper.startPeriodic(period/1000);
        isRunning = true;
    }

    /**
     * Starts the loop notifier running with a target goal heading.
     * 
     * @param goalHeading
     */
    public void start(double goalHeading) {
        resetGoalHeading(goalHeading);
        looper.startPeriodic(period/1000);
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
        goalHeading = goalHeading + nearestHeadingEquivalent(heading)/2;
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

    @Override
    public void setSetpoint(double setpoint) {
        super.setSetpoint(setpoint);

        startPose = new Pose2d(0, 0, new Rotation2d(driveTrain.getHeadingDegrees()));
        endPose = new Pose2d(0, 0, new Rotation2d(setpoint));

        //trajectory = TrajectoryGenerator.generateTrajectory(startPose, new ArrayList<Translation2d>(), endPose, trajectoryConfig);

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
        if(relative>180){
            relative = relative-360;
        }
        double nearest = relative + curr;
        System.out.println("Nearest Equivalent of "+heading+" to "+curr+" is "+nearest);
        return nearest;
    }

    private double floormod(double num, double denom) {
        return ((num % denom) + denom) % denom;
    }
}