package frc.robot.commands.vision;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.NetworkTablePaths;

/**
 * General flow for this command:
 * 
 * Listener->Notifier->FollowPID-+ V Command->Execute->Evaluate Errors-> Pass to
 * Next Command
 * 
 */
public class PositionVisionCommand extends CommandBase {

    private final DriveTrain driveTrain;
    private final Shooter shooter;
    private final NetworkTableEntry angleEntry;
    private int angleListenerHandle;
    private final NetworkTableEntry distanceEntry;
    private int distanceListenerHandle;
    private boolean turning = false;
    private Notifier pidHandeler;
    private double distance;
    private boolean finished = false;

    private final double maxVisionTurn = 0.4;
    private final double period = 10;
    private final double velocityTolerance = 0.1;
    private final double errorTolerance = 0.1;

    private PIDController angleFollower;
    private final double kP = 0.2;
    private final double kI = 0.00001;
    private final double kD = 0.0;

    public PositionVisionCommand(Shooter shooter, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.shooter = shooter;

        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        angleEntry = ntInst.getEntry(NetworkTablePaths.shooterAngle);
        angleListenerHandle = angleEntry.addListener(this::listenAngle, EntryListenerFlags.kUpdate);
        distanceEntry = ntInst.getEntry(NetworkTablePaths.shooterDistance);
        distanceListenerHandle = distanceEntry.addListener(this::listenDistance, EntryListenerFlags.kUpdate);

        angleFollower = new PIDController(kP, kI, kD, period);
    }

    @Override
    public void initialize() {
        angleListenerHandle = angleEntry.addListener(this::listenAngle, EntryListenerFlags.kUpdate);
        distanceListenerHandle = distanceEntry.addListener(this::listenDistance, EntryListenerFlags.kUpdate);
        finished = false;
    }

    @Override
    public void execute() {
        if (Math.abs(angleFollower.getPositionError()) < errorTolerance
                && Math.abs(driveTrain.getRotVelocity()) < velocityTolerance) {
            turning = false;
        }
    }

    public void followPID() {
        double out = angleFollower.calculate(driveTrain.getRot());
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
        driveTrain.tankDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    private void listenAngle(EntryNotification n) {
        double val = n.value.getDouble();
        if (!turning && val > errorTolerance) {
            finished = false;
            turnByAngle(val);
        } else if (!turning) {
            finished = true;
        }
    }

    private void listenDistance(EntryNotification n) {
        distance = n.value.getDouble();
    }

    private void turnByAngle(double degrees) {
        if (!(pidHandeler == null)) {
            pidHandeler.stop();
        }
        turning = true;
        angleFollower.setSetpoint(degrees);
        driveTrain.resetRot();
        pidHandeler = new Notifier(() -> {
            followPID();
        });
        pidHandeler.startPeriodic(period);
    }

}