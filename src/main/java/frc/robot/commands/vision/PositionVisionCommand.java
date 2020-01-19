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
    private double angle;

    private final double maxVisionTurn = 0.4;
    private final double period = 10;

    private PIDController angleFollower;
    private final double kP = 0.2;
    private final double kI = 0.00001;
    private final double kD = 0.0;

    public PositionVisionCommand(Shooter shooter, DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        this.shooter = shooter;

        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        NetworkTable table = ntInst.getTable("Vision");
        angleEntry = table.getEntry("angle");
        angleListenerHandle = angleEntry.addListener(this::listenAngle, EntryListenerFlags.kUpdate);
        distanceEntry = table.getEntry("distance");
        distanceListenerHandle = distanceEntry.addListener(this::listenDistance, EntryListenerFlags.kUpdate);

        angleFollower = new PIDController(kP, kI, kD, period);
    }

    @Override
    public void initialize() {
        angleListenerHandle = angleEntry.addListener(this::listenAngle, EntryListenerFlags.kUpdate);
        distanceListenerHandle = distanceEntry.addListener(this::listenDistance, EntryListenerFlags.kUpdate);
    }

    @Override
    public void execute() {

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
        try{
        pidHandeler.stop();
        }catch(NullPointerException e){
            // do nothing
        }
    }

    @Override
    public boolean isFinished() {
        return !turning && angle <= 0.01;
    }

    private void listenAngle(EntryNotification n) {
        double val = n.value.getDouble();
        if (!turning && val > 0.01) {
            turnByAngle(val);
        }
    }

    private void listenDistance(EntryNotification n) {
        distance = n.value.getDouble();
    }

    private void turnByAngle(double degrees) {
        if (!(pidHandeler == null)) {
            pidHandeler.stop();
        }
        angle = degrees;
        turning = true;
        angleFollower.setSetpoint(degrees);
        driveTrain.resetRot();
        pidHandeler = new Notifier(() -> {
            followPID();
        });
        pidHandeler.startPeriodic(period);
    }

}