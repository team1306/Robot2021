package frc.robot.commands.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.NetworkTablePaths;

public class PositionVisionCommand extends CommandBase {

    private final DriveTrain driveTrain;
    private final Shooter shooter;
    
    private final NetworkTableEntry angleEntry;
    private int angleListenerHandle;
    private final NetworkTableEntry distanceEntry;
    private int distanceListenerHandle;
    private Notifier pidHandeler;
    private double distance;
    private final NetworkTableEntry putHeading;

    private final double maxVisionTurn = 0.4;
    private final double period = 10; // 10 ms
    private final double velocityTolerance = 0.1;
    private final double errorTolerance = 0.1;

    private PIDController angleFollower;
    private final double kP = 0.2;
    private final double kI = 0.00001;
    private final double kD = 0.0;

    private double angle = Double.MAX_VALUE;// This records angle offset from vision target. Set to something outside of
                                            // tolerance range. Since the PIDController doesn't actually listen to this
                                            // variable for turning, it is safe to set.

    public PositionVisionCommand(Shooter shooter, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.shooter = shooter;

        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        angleEntry = ntInst.getEntry(NetworkTablePaths.shooterAngle);
        angleListenerHandle = angleEntry.addListener(this::listenAngle, EntryListenerFlags.kUpdate);
        distanceEntry = ntInst.getEntry(NetworkTablePaths.shooterDistance);
        distanceListenerHandle = distanceEntry.addListener(this::listenDistance, EntryListenerFlags.kUpdate);

        putHeading = ntInst.getEntry(NetworkTablePaths.robotHeading);

        angleFollower = new PIDController(kP, kI, kD, period);
    }

    @Override
    public void initialize() {
        angleListenerHandle = angleEntry.addListener(this::listenAngle, EntryListenerFlags.kUpdate);
        distanceListenerHandle = distanceEntry.addListener(this::listenDistance, EntryListenerFlags.kUpdate);
        angle = Double.MAX_VALUE;
        driveTrain.shift(DriveTrain.K_LOW_GEAR);
    }

    @Override
    public void execute() {
        putHeading.setDouble(driveTrain.getHeadingDegrees());
    }

    /**
     * Called by the PID Handeler.
     */
    public void followPID() {
        double out = angleFollower.calculate(driveTrain.getHeadingDegrees());
        out = Math.min(Math.max(-maxVisionTurn, out), maxVisionTurn);
        driveTrain.tankDrive(-out, out);
    }

    @Override
    public void end(boolean interupted) {
        angleEntry.removeListener(angleListenerHandle);
        distanceEntry.removeListener(distanceListenerHandle);
        try {
            pidHandeler.stop();
        } catch (NullPointerException e) {
            // do nothing
        }
        // reset speeds
        driveTrain.tankDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        // when not moving and both error and the vision-produced angle is under
        // tolerance
        return Math.abs(driveTrain.getRotVelocity()) < velocityTolerance
                && Math.abs(angleFollower.getPositionError()) + Math.abs(angle) < errorTolerance;
    }

    private void listenAngle(EntryNotification n) {
        double val = n.value.getDouble();
        double heading = val;
        turnToHeading(heading);
    }

    private void listenDistance(EntryNotification n) {
        distance = n.value.getDouble();
        shooter.targetDistance(distance);
    }

    /**
     * Starts the processes for following based on PID
     * 
     * @param degrees - current angular offset (in degrees)
     */
    private void turnByAngle(double degrees) {
        turnToHeading(driveTrain.getHeadingDegrees() + degrees);
    }

    /**
     * Starts the processes for following the PID to a specific gyro heading
     * 
     * @param heading - the goal heading (in degrees)
     */
    private void turnToHeading(double heading) {
        if (!(pidHandeler == null)) {
            pidHandeler.stop();
        }
        double currHeading = driveTrain.getHeadingDegrees();
        angle = heading - currHeading;// variable records angle angle offset - not heading
        if (this.isScheduled()) {
            angleFollower.setSetpoint(heading);
            pidHandeler = new Notifier(() -> {
                followPID();
            });
            pidHandeler.startPeriodic(period);
        }
    }

}